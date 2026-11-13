"""Experiment 2: gesture control.

`GestureAnalyzer` turns per-frame hand landmarks into *events*: a gesture has to be held (static ones) or performed
(beckoning = repeated finger curl, pushing away = the hand grows towards the camera) over a time window, fires once and
is then latched until the hand is released, so holding a gesture never repeats the command.

Image-direction convention: pointing towards the image left makes the robot turn left (`point_mirror` flips it).
"""
import math
import time
from collections import deque

import cv2
import numpy as np

from .common import Experiment, base_limits, draw_people, put_text
from .follow import FollowRunner
from .motion import body_delta, translate, turn_by

FINGERS = ((8, 6), (12, 10), (16, 14), (20, 18))        # (tip, pip) of index..pinky
MOTION_GESTURES = ("come", "away", "left", "right")
ALL_GESTURES = ("stop", "come", "away", "left", "right", "resume")
HAND_EDGES = ((0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (5, 9), (9, 10), (10, 11), (11, 12),
              (9, 13), (13, 14), (14, 15), (15, 16), (13, 17), (17, 18), (18, 19), (19, 20), (0, 17))


def hand_features(h):
    """Static descriptors of one hand from landmarks (pixels)."""
    pts = h["pts"]
    wrist = pts[0]
    scale = float(np.linalg.norm(pts[9] - wrist)) + 1e-6
    ext = [bool(np.linalg.norm(pts[t] - wrist) > 1.1 * np.linalg.norm(pts[p] - wrist)) for t, p in FINGERS]
    curl = float(np.mean([np.linalg.norm(pts[t] - wrist) for t, _ in FINGERS]) / scale)
    if h.get("label") == "Thumb_Up" and not any(ext):
        kind = "thumb"
    elif all(ext):
        kind = "palm"
    elif ext[0] and not any(ext[1:]):
        kind = "point"
    else:
        kind = "other"
    v = pts[8] - pts[5]
    return dict(kind=kind, curl=curl, scale=scale, wrist=wrist.copy(), pdir=v, depth=h.get("depth"), label=h.get("label"))


class GestureAnalyzer:
    def __init__(self, cfg):
        self.c = cfg
        self.frames = deque()                  # (t, features or None)
        self.latched = None                    # gesture name waiting for the hand to be released
        self.released_since = None
        self.cooldown_until = 0.0
        self.last_event = None
        self.debug = {}
        self.history = deque(maxlen=800)       # (t, kind, curl, scale, wrist_x, wrist_y, depth) per camera frame
        self.event_log = deque(maxlen=60)      # (t, name)

    def reset(self):
        self.frames.clear()
        self.latched = None
        self.released_since = None

    def update(self, hands, now):
        c = self.c
        feat = None
        if hands:
            best = max(hands, key=lambda h: np.linalg.norm(h["pts"][9] - h["pts"][0]))     # closest/largest hand
            feat = hand_features(best)
        self.frames.append((now, feat))
        self.history.append((now, None if feat is None else feat["kind"], None if feat is None else float(feat["curl"]),
                             None if feat is None else float(feat["scale"]),
                             None if feat is None else float(feat["wrist"][0]), None if feat is None else float(feat["wrist"][1]),
                             None if feat is None else feat["depth"]))
        while self.frames and now - self.frames[0][0] > c["window_s"]:
            self.frames.popleft()
        recent = [(t, f) for t, f in self.frames if f is not None]
        self.debug = dict(kind=None if feat is None else feat["kind"], curl=None if feat is None else round(feat["curl"], 2),
                          scale=None if feat is None else round(feat["scale"], 1), n=len(recent))
        # ---- latch: one command per gesture instance
        if self.latched is not None:
            current = None if feat is None else feat["kind"]
            same = current is not None and current in self._kinds_of(self.latched)
            if same:
                self.released_since = None
            else:
                self.released_since = self.released_since or now
                if now - self.released_since >= c["release_s"]:
                    self.latched, self.released_since = None, None
            if self.latched is not None:
                return None
        if now < self.cooldown_until or feat is None:
            return None
        event = (self._push(recent, now) or self._beckon(recent, now) or self._stop(recent, now)
                 or self._resume(recent, now) or self._point(recent, now))
        if event:
            self.latched = event
            self.cooldown_until = now + c["cooldown_s"]
            self.last_event = (event, now)
            self.event_log.append((now, event))
            self.frames.clear()
        return event

    @staticmethod
    def _kinds_of(name):
        return {"stop": ("palm",), "away": ("palm",), "come": ("palm", "other"), "resume": ("thumb",),
                "left": ("point",), "right": ("point",)}[name]

    def _span(self, recent, kind, seconds, now):
        """Frames of `kind` covering the last `seconds` without interruption (>= 3 frames), else None."""
        window = [(t, f) for t, f in recent if now - t <= seconds + 0.25]
        if len(window) < 3 or window[-1][0] < now - 0.45:
            return None
        if any(f["kind"] != kind for _, f in window if now - _ <= seconds):
            return None
        if window[-1][0] - window[0][0] < 0.85 * seconds:
            return None
        return [f for _, f in window]

    def _stop(self, recent, now):
        c = self.c
        fr = self._span(recent, "palm", c["hold_s"], now)
        if fr is None:
            return None
        move = np.linalg.norm(fr[-1]["wrist"] - fr[0]["wrist"]) / fr[0]["scale"]
        growth = fr[-1]["scale"] / fr[0]["scale"]
        curls = [f["curl"] for f in fr]
        if move > 1.6 or growth > 1.18 or max(curls) - min(curls) > 0.3:      # moving, pushing or beckoning: not a still palm
            return None
        return "stop"

    def _resume(self, recent, now):
        return "resume" if self._span(recent, "thumb", self.c["hold_s"], now) else None

    def _point(self, recent, now):
        c = self.c
        fr = self._span(recent, "point", c["hold_s"], now)
        if fr is None:
            return None
        v = np.mean([f["pdir"] for f in fr], axis=0)
        if abs(v[0]) < c["point_min_ratio"] * abs(v[1]):
            return None                                                     # pointing up/down is not a command
        left_in_image = v[0] < 0
        left = left_in_image != bool(c["point_mirror"])
        return "left" if left else "right"

    def _push(self, recent, now):
        """Open palm moving towards the camera: the apparent hand size grows (and the depth shrinks)."""
        c = self.c
        win = [(t, f) for t, f in recent if now - t <= c["push_window_s"] and f["kind"] == "palm"]
        if len(win) < 3 or win[-1][0] < now - 0.4:
            return None
        growth = win[-1][1]["scale"] / win[0][1]["scale"]
        d0, d1 = win[0][1]["depth"], win[-1][1]["depth"]
        closer = d0 is not None and d1 is not None and d0 - d1 > 0.15
        self.debug["growth"] = round(growth, 2)
        return "away" if growth >= c["push_growth"] or (closer and growth > 1.15) else None

    def _beckon(self, recent, now):
        """Fingers repeatedly curl and open with the wrist roughly still: `beckon_cycles` full cycles in the window."""
        c = self.c
        if len(recent) < 5 or recent[-1][0] < now - 0.45:
            return None
        curls = [f["curl"] for _, f in recent]
        wr = np.array([f["wrist"] for _, f in recent])
        scale = np.mean([f["scale"] for _, f in recent])
        if np.ptp(wr[:, 0]) / scale > 2.2 or np.ptp(wr[:, 1]) / scale > 2.2:
            return None                                                     # waving or walking hand, not beckoning
        state, cycles = None, 0
        for v in curls:
            if v >= c["curl_hi"]:
                if state == "lo":
                    cycles += 1
                state = "hi"
            elif v <= c["curl_lo"] and state == "hi":
                state = "lo"                                                # a curl counts only after an open hand
        self.debug["cycles"] = cycles
        return "come" if cycles >= c["beckon_cycles"] else None


class GestureExperiment(Experiment):
    key = "e2_gesture"
    title = "2. Управление жестами"

    def __init__(self, ctx):
        super().__init__(ctx)
        self.gc = ctx.cfg["exp"]["gesture"]
        self.analyzer = GestureAnalyzer(self.gc)
        self.runner = None
        self.execute = True
        self.follow_active = True
        self.trial = None
        self.events = []
        self.unarmed = []
        self.last = None
        self.executing = None
        self._stamp = None

    def setup(self):
        self.gc = self.ecfg["gesture"]
        self.analyzer = GestureAnalyzer(self.gc)
        self.runner = FollowRunner(self.ctx)
        self.execute = bool(self.params.get("execute", True))
        self.follow_active = bool(self.params.get("follow", True)) and self.execute
        self.allow_runtime(float(self.params.get("runtime_s", 900)))
        self.events, self.unarmed, self.trial, self.last = [], [], None, None
        self.ctx.perception.tracker.reset()
        self.ctx.perception.set_want(people=True, hands=True, objects=False, pose=False)
        self.ctx.cfg["exp"]["detector"]["people_every"] = 2
        self.people_view()
        self.state = "running"
        self.say("жесты: %s; следование %s" % ("команды исполняются" if self.execute else "ТОЛЬКО распознавание (без движения)",
                                              "включено" if self.follow_active else "выключено"))

    def teardown(self):
        self.ctx.perception.set_want(people=False, hands=False)
        self.ctx.perception.crop = None
        self.ctx.cfg["exp"]["detector"]["people_every"] = 1
        self.io.stop_base()

    def run(self):
        f, g = self.ecfg["follow"], self.gc
        io = self.io
        with base_limits(io, max_vx=max(g["speed"], 0.15), max_vy=0.15, max_wz=max(g["turn_speed"], 0.6),
                         accel=0.5, angular_accel=2.0):
            while not self._stop.is_set():
                snap = self.ctx.perception.latest(max_age=1.0)
                now = time.monotonic()
                self._aim_crop(snap)
                event = None
                if snap is not None and snap.stamp != self._stamp:      # one analyzer step per camera frame
                    self._stamp = snap.stamp
                    event = self.analyzer.update(snap.hands, now)
                if event:
                    self.handle(event, now)
                elif self.follow_active and self.execute:
                    self.runner.tick(snap, now)
                    io.drive(*self.runner.cmd, hold=0.3)
                self.metrics = self.live_metrics()
                self.sleep(0.05)
                self._expire_trial()

    def _aim_crop(self, snap):
        """Search hands where the person is: crop the upper body of the nearest person (enlarged for the hand model)."""
        p = self.ctx.perception
        if snap is None or not self.gc["hand_crop"]:
            p.crop = None
            return
        people = [t for t in snap.people if t.misses == 0 and t.dist is not None]
        if not people:
            p.crop = None
            return
        t = min(people, key=lambda x: x.dist)
        x0, y0, x1, y1 = t.box
        w, h = x1 - x0, y1 - y0
        p.crop = (x0 - 0.9 * w, y0 - 0.1 * h, x1 + 0.9 * w, y0 + 0.75 * h)

    # ---- reactions (physical motion measured by odometry)
    def handle(self, event, now):
        io = self.io
        t_event = time.monotonic()
        pose0 = list(io.pose())
        armed = self.trial is not None and not self.trial.get("done")
        moving_before = any(abs(v) > 0.02 for v in self.runner.cmd) if self.runner else False
        rec = dict(gesture=event, t=round(t_event, 2), armed=armed, executed=self.execute, moving_before=moving_before)
        self.say("жест: %s%s" % (event, "" if self.execute else " (без движения)"))
        self.executing = event
        if self.execute:
            try:
                self.act(event)
            finally:
                io.stop_base() if event != "resume" else None
        self.executing = None
        io.wait(0.3)
        dx, dy, dyaw = body_delta(pose0, io.pose())
        rec.update(dx=round(dx, 3), dy=round(dy, 3), dyaw_deg=round(float(np.degrees(dyaw)), 1),
                   follow_active=self.follow_active)
        self.last = rec
        if armed:
            self.trial["events"].append(rec)
            self.finish_trial()
        else:
            self.unarmed.append(rec)
            self.ctx.store.add(self.key, dict(kind="unarmed_event", **rec))
        self.events.append(rec)
        self.analyzer.cooldown_until = time.monotonic() + self.gc["cooldown_s"]

    def act(self, event):
        io, g = self.io, self.gc
        if event == "stop":
            self.follow_active = False
            io.stop_base()
        elif event == "resume":
            self.follow_active = True
            self.runner.unlock()
        elif event == "come":
            self.follow_active = False
            snap = self.ctx.perception.latest(max_age=1.0)
            people = [t for t in (snap.people if snap else []) if t.misses == 0 and t.dist]
            want = g["step_m"] if not people else max(0.0, min(1.0, min(p.dist for p in people) - g["come_stop_m"]))
            if want > 0.05:
                translate(io, want, 0.0, speed=g["speed"], timeout=10.0)
        elif event == "away":
            self.follow_active = False
            translate(io, -g["step_m"], 0.0, speed=g["speed"], timeout=10.0)
        elif event in ("left", "right"):
            self.follow_active = False
            turn_by(io, math.radians(g["turn_deg"]) * (1 if event == "left" else -1), wz=g["turn_speed"])

    # ---- test protocol: the operator announces which gesture comes next, the robot's real motion is judged
    def command(self, name, **kw):
        if name == "trial_start":
            expect = kw.get("expect", "stop")
            self.trial = dict(expect=expect, t0=time.monotonic(), events=[], done=False, ctx=kw)
            self.analyzer.reset()
            self.say("испытание: ждём жест «%s» (%.0f с)" % (expect, self.gc["trial_window_s"]))
        elif name == "trial_cancel":
            self.trial = None
        elif name == "execute":
            self.execute = bool(kw.get("on", True))
            if not self.execute:
                self.follow_active = False
                self.io.stop_base()
        elif name == "follow":
            self.follow_active = bool(kw.get("on", True)) and self.execute
            if self.follow_active:
                self.runner.unlock()
        return True

    def _expire_trial(self):
        t = self.trial
        if t and not t["done"] and time.monotonic() - t["t0"] > self.gc["trial_window_s"] + 4.0 and not self.executing:
            if not t["events"]:
                self.finish_trial(timeout=True)

    def finish_trial(self, timeout=False):
        t = self.trial
        if t is None or t["done"]:
            return
        t["done"] = True
        expect = t["expect"]
        ev = t["events"][0] if t["events"] else None
        got = None if ev is None else ev["gesture"]
        motion_ok = None
        if ev is not None and self.execute:
            motion_ok = {
                "come": ev["dx"] >= 0.08, "away": ev["dx"] <= -0.08, "left": ev["dyaw_deg"] >= 0.5 * self.gc["turn_deg"],
                "right": ev["dyaw_deg"] <= -0.5 * self.gc["turn_deg"],
                "stop": abs(ev["dx"]) < 0.06 and abs(ev["dyaw_deg"]) < 6 and not self.follow_active,
                "resume": self.follow_active,
            }[got]
        correct = got == expect and (motion_ok is not False)
        rec = dict(kind="trial", expect=expect, got=got, correct=bool(correct), motion_ok=motion_ok, timeout=timeout,
                   extra_events=len(t["events"]) - 1 if t["events"] else 0, executed=self.execute,
                   dx=None if ev is None else ev["dx"], dyaw_deg=None if ev is None else ev["dyaw_deg"],
                   distance_m=t["ctx"].get("distance_m"), background=t["ctx"].get("background"),
                   light=t["ctx"].get("light"), video=self._recorder.path if self._recorder else None)
        self.ctx.store.add(self.key, rec)
        self.say("итог: ожидали «%s», получили «%s», движение %s → %s" % (
            expect, got, motion_ok, "ВЕРНО" if correct else "ОШИБКА"))
        self.trial = None

    def live_metrics(self):
        trials = [r for r in self.ctx.store.load(self.key) if r.get("kind") == "trial"]
        stats = {}
        for g in ALL_GESTURES:
            rows = [r for r in trials if r["expect"] == g]
            stats[g] = dict(n=len(rows), ok=sum(1 for r in rows if r["correct"]))
        dangerous = [r for r in self.unarmed if r["gesture"] in MOTION_GESTURES and r["executed"]]
        total = sum(s["n"] for s in stats.values())
        ok = sum(s["ok"] for s in stats.values())
        return dict(hand=self.analyzer.debug, last=self.last, follow=self.follow_active, execute=self.execute,
                    stats=stats, total=total, ok=ok, rate=None if not total else round(100 * ok / total, 1),
                    false_events=len(self.unarmed), dangerous_false=len(dangerous),
                    trial=None if not self.trial else dict(expect=self.trial["expect"],
                                                          left=round(self.gc["trial_window_s"] - (time.monotonic() - self.trial["t0"]), 1)),
                    runner=None if self.runner is None else self.runner.state, fps=round(self.ctx.perception.hz, 1))

    def overlay(self, img):
        snap = self.ctx.perception.latest()
        if snap is not None:
            draw_people(img, snap.people, self.runner.lock.id if self.runner else None)
            for h in snap.hands:
                pts = h["pts"].astype(int)
                for a, b in HAND_EDGES:
                    cv2.line(img, tuple(pts[a]), tuple(pts[b]), (0, 255, 255), 2)
                put_text(img, "%s %s" % (h["label"], "" if h["depth"] is None else "%.2f m" % h["depth"]),
                         (int(pts[0][0]), int(pts[0][1]) + 18), (0, 255, 255))
            if self.ctx.perception.crop:
                x0, y0, x1, y1 = [int(v) for v in self.ctx.perception.crop]
                cv2.rectangle(img, (x0, y0), (x1, y1), (255, 120, 0), 1)
        m = self.metrics or {}
        put_text(img, "E2 hand=%s follow=%s exec=%s" % (m.get("hand", {}).get("kind"), m.get("follow"), m.get("execute")),
                 (8, 20), (0, 255, 255))
        if self.last:
            put_text(img, "last: %s" % self.last["gesture"], (8, 44), (0, 255, 0), 0.8, 2)
        if self.trial:
            put_text(img, "TRIAL expect=%s" % self.trial["expect"], (8, 70), (0, 200, 255), 0.7, 2)
        return img
