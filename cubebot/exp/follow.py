"""Experiment 1: person following.

`FollowRunner` is the reusable part (detector snapshot in -> body twist out); experiments 2 and 5 drive it too.
Safety rules baked in: no measurement fresher than `stale_s` -> zero twist; no depth and no usable box -> zero twist;
the lidar guard of BaseIO still sits behind every command; the search for a lost person is bounded in time.
"""
import math
import time
from collections import deque

import numpy as np

from ..geometry import wrap
from .common import Experiment, TargetLock, base_limits, draw_people, put_text


def clip(v, lo, hi):
    return max(lo, min(hi, v))


class FollowRunner:
    IDLE, LOCKING, FOLLOW, HOLD, LOST, GAVE_UP = "idle", "locking", "following", "holding", "lost", "gave_up"

    def __init__(self, ctx):
        self.ctx = ctx
        self.io = ctx.io
        self.cfg = ctx.cfg["exp"]["follow"]
        self.lock = TargetLock(ctx.cfg["exp"]["tracker"])
        self.state = self.IDLE
        self.target = None
        self.cmd = (0.0, 0.0, 0.0)
        self.lost_since = None
        self.losses = 0
        self.search = None
        self.samples = []                       # (t, dist, bearing) while the target is tracked
        self._walk = deque(maxlen=40)           # (t, x, y) of the target in the odom frame, for the speed feed-forward
        self.history = deque(maxlen=800)        # (t, dist, bearing_deg, vx, vy, wz, state) for the live charts
        self.note = ""

    # ---- target management
    def unlock(self):
        self.lock.clear()
        self.target = None
        self.state = self.IDLE

    def try_lock(self, snap, now):
        """Lock the person who is close enough and nearest to the image centre (the one standing in front)."""
        c = self.cfg
        best, best_key = None, 1e9
        cx = snap.frame.rgb.shape[1] / 2
        for tr in snap.people:
            if tr.misses or tr.dist is None or tr.hits < self.ctx.cfg["exp"]["tracker"]["min_hits_to_lock"]:
                continue
            if tr.dist > c["lock_max_dist_m"] or tr.mono:
                continue
            key = abs((tr.box[0] + tr.box[2]) / 2 - cx)
            if key < c["lock_center_px"] and key < best_key:
                best, best_key = tr, key
        if best is None:
            return False
        self.lock.lock(best, now)
        self.note = "locked on #%d at %.2f m" % (best.id, best.dist)
        self.state = self.FOLLOW
        return True

    # ---- control law
    def twist(self, tr):
        """Body twist that brings the person to the stand-off distance and keeps them centred."""
        c = self.cfg
        tx, ty = float(tr.xyz_base[0]), float(tr.xyz_base[1])
        r = math.hypot(tx, ty)
        if r < 0.15:
            return 0.0, 0.0, 0.0
        b = math.atan2(ty, tx)
        e = r - c["standoff_m"]
        if r < c["min_dist_m"]:
            speed = -c["max_back"]
        elif abs(e) <= c["deadband_m"]:
            speed = 0.0
        else:
            speed = clip(c["kp_x"] * (e - math.copysign(c["deadband_m"], e)), -c["max_back"], c["max_vx"])
        if tr.mono:
            speed *= 0.5                                # distance from the box height only: be careful
        if 0.01 < abs(speed) < 0.035:
            speed = math.copysign(0.035, speed)         # the wheels stall below ~0.03 m/s
        ux, uy = tx / r, ty / r
        gate = clip(1.0 - (abs(b) - 0.30) / 0.30, 0.0, 1.0)      # large bearing: turn first, then translate
        vx, vy = speed * ux * gate, speed * uy * gate
        vy = clip(vy, -c["max_vy"], c["max_vy"])
        wz = c["kp_wz"] * b if abs(b) > 0.04 else 0.0
        if wz and abs(wz) < 0.12:
            wz = math.copysign(0.12, wz)                # wheels stall below ~0.1 rad/s
        wz = clip(wz, -c["max_wz"], c["max_wz"])
        return self.steer_around(vx, vy, wz)

    def feedforward(self, tr, now):
        """Target velocity in the base frame from its odom-frame track (least squares over ~0.8 s).

        Without it a pure proportional law trails a walking person by v/k_x + dead-band (0.5 m at 0.25 m/s)."""
        x, y, yaw = self.io.pose()
        c, s = math.cos(yaw), math.sin(yaw)
        wx = x + c * float(tr.xyz_base[0]) - s * float(tr.xyz_base[1])
        wy = y + s * float(tr.xyz_base[0]) + c * float(tr.xyz_base[1])
        if not self._walk or now - self._walk[-1][0] > 1.0:
            self._walk.clear()
        self._walk.append((now, wx, wy))
        pts = [p for p in self._walk if now - p[0] <= 0.8]
        if len(pts) < 4 or pts[-1][0] - pts[0][0] < 0.45:
            return 0.0, 0.0
        t = np.array([p[0] for p in pts]) - pts[0][0]
        vx_w, vy_w = np.polyfit(t, [p[1] for p in pts], 1)[0], np.polyfit(t, [p[2] for p in pts], 1)[0]
        sp = math.hypot(vx_w, vy_w)
        if sp < 0.10 or sp > 1.5:
            return 0.0, 0.0
        k = self.cfg.get("ff_gain", 0.9)
        return k * (c * vx_w + s * vy_w), k * (-s * vx_w + c * vy_w)

    def steer_around(self, vx, vy, wz):
        """Mecanum sidestep around lidar obstacles on the way to the person; stop translating when boxed in."""
        speed = math.hypot(vx, vy)
        pts = self.io.scan_points()
        if speed < 0.03 or pts is None:
            return vx, vy, wz
        guard, clear = self.io.guard, self.cfg["avoid_clear_m"]
        heading = math.atan2(vy, vx)
        best = None
        for off in (0.0, 0.5, -0.5, 1.0, -1.0, 1.5, -1.5):
            d = (math.cos(heading + off), math.sin(heading + off))
            if guard._free_distance(pts, d, clear + 0.15) >= clear:
                best = off
                break
        if best is None:
            self.note = "boxed in by obstacles"
            return 0.0, 0.0, wz
        if best != 0.0:
            self.note = "sidestepping an obstacle (%+.0f deg)" % math.degrees(best)
            speed *= 0.7
        return speed * math.cos(heading + best), speed * math.sin(heading + best), wz

    # ---- one control cycle
    def tick(self, snap, now, allow_motion=True):
        """Returns the commanded twist. `snap` may be None when no fresh perception exists."""
        cmd = self._tick(snap, now, allow_motion)
        t = self.target
        self.history.append((now, None if t is None else t.dist, None if t is None or t.bearing is None else math.degrees(t.bearing),
                             cmd[0], cmd[1], cmd[2], self.state))
        return cmd

    def _tick(self, snap, now, allow_motion):
        c = self.cfg
        tracks = snap.people if snap is not None else []
        tr = self.lock.update(tracks, now) if self.lock.locked else None
        if not self.lock.locked and snap is not None and self.state in (self.IDLE, self.LOCKING):
            self.state = self.LOCKING
            if not self.try_lock(snap, now):
                self.cmd = (0.0, 0.0, 0.0)
                return self.cmd
            tr = self.lock.update(tracks, now)
        fresh = tr is not None and tr.misses == 0 and tr.xyz_base is not None and now - tr.seen_t < c["stale_s"]
        self.target = tr if fresh else None
        if fresh:
            if self.state in (self.LOST,):
                self.note = "target found again (#%d)" % tr.id
            self.lost_since, self.search = None, None
            cmd = self.twist(tr)
            fx, fy = self.feedforward(tr, now)
            if (fx or fy) and tr.dist is not None and tr.dist >= self.cfg["min_dist_m"]:
                gate = clip(1.0 - (abs(tr.bearing or 0.0) - 0.30) / 0.30, 0.0, 1.0)
                vx = clip(cmd[0] + fx * gate, -self.cfg["max_back"], self.cfg["max_vx"])
                vy = clip(cmd[1] + fy * gate, -self.cfg["max_vy"], self.cfg["max_vy"])
                cmd = self.steer_around(vx, vy, cmd[2])
            self.state = self.HOLD if cmd == (0.0, 0.0, 0.0) or (abs(cmd[0]) + abs(cmd[1]) < 0.01 and abs(cmd[2]) < 0.05) else self.FOLLOW
            self.samples.append((now, tr.dist, tr.bearing, tr.mono))
            self.cmd = cmd if allow_motion else (0.0, 0.0, 0.0)
            return self.cmd
        # no reliable measurement: never move on stale data
        if self.state in (self.FOLLOW, self.HOLD):
            self.state = self.LOST
            self._walk.clear()
            self.losses += 1
            self.lost_since = now
            self.search = None
            self.note = "target lost: stopped, searching"
        if self.state == self.LOST:
            self.cmd = self.search_motion(now) if allow_motion else (0.0, 0.0, 0.0)
            return self.cmd
        self.cmd = (0.0, 0.0, 0.0)
        return self.cmd

    def search_motion(self, now):
        """Bounded search: turn towards where the person left the view, sweep back and forth, then give up."""
        c = self.cfg
        if now - self.lost_since > c["search_s"]:
            self.state = self.GAVE_UP
            self.note = "target not found within %.0f s" % c["search_s"]
            return 0.0, 0.0, 0.0
        yaw = self.io.pose()[2]
        if self.search is None:
            side = 1.0 if self.lock.last_bearing >= 0 else -1.0
            self.search = dict(origin=yaw, legs=[side * 0.9, -side * 1.8, side * 0.9], leg=0, base=yaw)
        s = self.search
        if now - self.lost_since < 0.8:                  # give the tracker a moment before moving the camera
            return 0.0, 0.0, 0.0
        goal = wrap(s["origin"] + sum(s["legs"][:s["leg"] + 1]))
        err = wrap(goal - yaw)
        if abs(err) < 0.08:
            s["leg"] = (s["leg"] + 1) % len(s["legs"])
            return 0.0, 0.0, 0.0
        return 0.0, 0.0, math.copysign(c["search_wz"], err)


class FollowExperiment(Experiment):
    key = "e1_follow"
    title = "1. Следование за человеком"
    TRIALS = ("straight", "turn90", "stop", "occlusion", "two_people")

    def __init__(self, ctx):
        super().__init__(ctx)
        self.runner = FollowRunner(ctx)
        self.trial = None
        self.trials_done = []
        self._tick_log = 0.0

    def setup(self):
        self.runner = FollowRunner(self.ctx)
        self.allow_runtime(float(self.params.get("runtime_s", 600)))
        f = self.ecfg["follow"]
        if "standoff_m" in self.params:
            f["standoff_m"] = float(self.params["standoff_m"])
        self.ctx.perception.tracker.reset()
        self.ctx.perception.set_want(people=True, objects=False, hands=False, pose=False)
        self.people_view()
        self.say("камера смотрит вперёд; встаньте перед роботом на 1-3 м (захват цели автоматический)")
        self.state = "running"

    def teardown(self):
        self.ctx.perception.set_want(people=False)
        self.end_trial({"verdict": "aborted"}) if self.trial else None
        self.io.stop_base()

    def run(self):
        f = self.ecfg["follow"]
        ctx, io = self.ctx, self.io
        with base_limits(io, max_vx=f["max_vx"], max_vy=f["max_vy"], max_wz=f["max_wz"], accel=f["accel"],
                         angular_accel=f["angular_accel"]):
            while not self._stop.is_set():
                snap = ctx.perception.latest(max_age=1.0)
                now = time.monotonic()
                self.runner.tick(snap, now)
                vx, vy, wz = self.runner.cmd
                io.drive(vx, vy, wz, hold=0.3)
                if self.runner.state == FollowRunner.GAVE_UP:
                    self.say("цель потеряна и не найдена: остановка, жду нового человека перед роботом")
                    io.stop_base()
                    self.runner.state = FollowRunner.IDLE
                    self.runner.unlock()
                self.metrics = self.live_metrics()
                self.sleep(0.05)

    # ---- trial bookkeeping (operator starts/ends trials; numbers are measured automatically)
    def command(self, name, **kw):
        if name == "lock":
            self.runner.unlock()
            self.say("повторный захват ближайшего человека перед роботом")
        elif name == "unlock":
            self.runner.unlock()
        elif name == "trial_start":
            self.start_trial(kw.get("type", "straight"))
        elif name == "trial_end":
            self.end_trial(kw)
        return True

    def start_trial(self, kind):
        r = self.runner
        self.trial = dict(type=kind, t0=time.monotonic(), losses0=r.losses, reacq0=r.lock.reacquired,
                          rejected0=r.lock.rejected, pose0=list(self.io.pose()), n0=len(r.samples), id0=r.lock.id)
        self.say("испытание «%s» началось" % kind)

    def end_trial(self, verdict):
        tr = self.trial
        if not tr:
            return
        self.trial = None
        r = self.runner
        s = [x for x in r.samples[tr["n0"]:] if not x[3]]
        standoff = self.ecfg["follow"]["standoff_m"]
        dist = np.array([x[1] for x in s]) if s else np.array([])
        pose1 = self.io.pose()
        p0 = tr["pose0"]
        rec = dict(
            type=tr["type"], seconds=round(time.monotonic() - tr["t0"], 1), standoff_m=standoff,
            samples=len(s), dist_mean=None if not len(dist) else round(float(dist.mean()), 3),
            dist_min=None if not len(dist) else round(float(dist.min()), 3),
            dist_max=None if not len(dist) else round(float(dist.max()), 3),
            dist_rms_error=None if not len(dist) else round(float(np.sqrt(np.mean((dist - standoff) ** 2))), 3),
            within_035=None if not len(dist) else round(float(np.mean(np.abs(dist - standoff) < 0.35)), 2),
            target_losses=r.losses - tr["losses0"], reacquired=r.lock.reacquired - tr["reacq0"],
            other_people_rejected=r.lock.rejected - tr["rejected0"], track_id_start=tr["id0"], track_id_end=r.lock.id,
            robot_travel_m=round(math.hypot(pose1[0] - p0[0], pose1[1] - p0[1]), 2),
            state_end=r.state,
            measured_distance_m=verdict.get("measured_distance_m"), estimated_distance_end=None if not len(dist) else round(float(dist[-1]), 3),
            correct_target=verdict.get("correct_target"), operator_verdict=verdict.get("verdict"),
            note=verdict.get("note", ""), video=self._recorder.path if self._recorder else None)
        checks = [rec["target_losses"] <= 1 or rec["type"] == "occlusion", rec["state_end"] in ("following", "holding"),
                  (rec["within_035"] or 0) >= 0.7 or rec["type"] in ("stop",)]
        rec["auto_pass"] = bool(all(checks))
        self.ctx.store.add(self.key, rec)
        self.trials_done.append(rec)
        self.say("испытание записано: потерь цели %d, дистанция %.2f м (rms ошибка %s)" % (
            rec["target_losses"], rec["dist_mean"] or 0, rec["dist_rms_error"]))

    def live_metrics(self):
        r = self.runner
        t = r.target
        return dict(state=r.state, note=r.note, dist=None if t is None else round(t.dist, 2),
                    bearing_deg=None if t is None or t.bearing is None else round(math.degrees(t.bearing), 1),
                    mono=None if t is None else t.mono, cmd=[round(v, 2) for v in r.cmd], losses=r.losses,
                    reacquired=r.lock.reacquired, rejected=r.lock.rejected, target_id=r.lock.id,
                    people=len([x for x in (self.ctx.perception.latest() or _Empty).people if x.misses == 0]),
                    trial=None if not self.trial else self.trial["type"], fps=round(self.ctx.perception.hz, 1))

    def overlay(self, img):
        snap = self.ctx.perception.latest()
        r = self.runner
        if snap is not None:
            draw_people(img, snap.people, r.lock.id)
        m = self.metrics or {}
        put_text(img, "E1 %s dist=%s cmd=%s losses=%s" % (r.state, m.get("dist"), m.get("cmd"), r.losses), (8, 20),
                 (0, 255, 255))
        if r.note:
            put_text(img, r.note, (8, 42), (255, 255, 0))
        if self.trial:
            put_text(img, "TRIAL: %s" % self.trial["type"], (8, 64), (0, 200, 255))
        return img


class _Empty:
    people = []
