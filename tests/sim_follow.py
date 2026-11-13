"""Real-time closed-loop simulation of FollowExperiment (real tracker, lock, controller, experiment thread).

Run:  python tests/sim_follow.py [scenario ...]        (about 25 s per scenario, no robot needed)
The world is synthetic (people = coloured rectangles, perfect depth, detector at 8 Hz); it checks the control logic and the
identity handling, NOT the physical robot. It exists to catch logic bugs before the physical trials.
"""
import math
import os
import sys
import tempfile
import threading
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from cubebot import config  # noqa: E402
from cubebot.exp.common import Context, PersonTracker, ResultStore, Snapshot, box_iou  # noqa: E402
from cubebot.exp.follow import FollowExperiment  # noqa: E402
from cubebot.robot_io import MissionAbort  # noqa: E402

K = np.array([[480.7, 0, 320.0], [0, 480.7, 240.0], [0, 0, 1.0]])


class World:
    def __init__(self, people):
        self.people = people                     # dict name -> dict(color, pos(t) -> (x, y) or None)
        self.t0 = time.monotonic()
        self.robot = np.zeros(3)
        self.vel = np.zeros(3)
        self.cmd = (0.0, 0.0, 0.0)
        self.until = 0.0
        self.log = []
        self.lim = None
        self.run = True
        threading.Thread(target=self._phys, daemon=True).start()

    def t(self):
        return time.monotonic() - self.t0

    def _phys(self):
        dt = 0.02
        while self.run:
            time.sleep(dt)
            desired = self.cmd if time.monotonic() < self.until else (0, 0, 0)
            lim = [self.lim["max_vx"], self.lim["max_vy"], self.lim["max_wz"]]
            acc = [self.lim["accel"], self.lim["accel"], self.lim["angular_accel"]]
            for i in range(3):
                tgt = max(-lim[i], min(lim[i], desired[i]))
                self.vel[i] += max(-acc[i] * dt, min(acc[i] * dt, tgt - self.vel[i]))
            c, s = math.cos(self.robot[2]), math.sin(self.robot[2])
            self.robot += dt * np.array([c * self.vel[0] - s * self.vel[1], s * self.vel[0] + c * self.vel[1], self.vel[2]])
            self.log.append((self.t(), *self.robot, *self.vel))

    def person_in_base(self, name):
        p = self.people[name]["pos"](self.t())
        if p is None:
            return None
        c, s = math.cos(self.robot[2]), math.sin(self.robot[2])
        dx, dy = p[0] - self.robot[0], p[1] - self.robot[1]
        return np.array([c * dx + s * dy - 0.0, -s * dx + c * dy])


class FakeShaper:
    def __init__(self, cfg):
        self.cfg = cfg

    def reset(self):
        pass


class FakeGuard:
    last_block = None
    mode = "normal"

    @staticmethod
    def _free_distance(points, direction, reach):
        return reach


class FakeArm:
    def camera_pose(self, servo):
        return np.eye(4)


class FakeIO:
    def __init__(self, cfg, world):
        self.cfg, self.world = cfg, world
        self.shaper = FakeShaper(cfg["base"])
        world.lim = cfg["base"]
        self.guard = FakeGuard()
        self.arm = FakeArm()
        self.abort_reason, self.started, self.last_output = None, None, (0, 0, 0)
        self.scan = None

    def now(self):
        return time.monotonic()

    def wait(self, s):
        end = time.monotonic() + s
        while time.monotonic() < end:
            if self.abort_reason:
                raise MissionAbort(self.abort_reason)
            time.sleep(min(0.02, max(0.0, end - time.monotonic())))
        if self.abort_reason:
            raise MissionAbort(self.abort_reason)

    def drive_until(self, command_fn, done_fn, timeout, label=""):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if done_fn():
                self.stop_base()
                return True
            self.drive(*command_fn())
            self.wait(0.05)
        self.stop_base()
        return False

    def request_abort(self, r):
        self.abort_reason = self.abort_reason or r

    def drive(self, vx, vy, wz, hold=0.3):
        self.world.cmd, self.world.until = (vx, vy, wz), time.monotonic() + hold
        self.last_output = (vx, vy, wz)

    def stop_base(self):
        self.world.cmd = (0.0, 0.0, 0.0)

    def pose(self):
        return tuple(self.world.robot)

    def scan_points(self):
        return None

    def arm_command(self):
        return [90, 90, 90, 10, 90, 30]

    def move_arm(self, *a, **k):
        pass

    def _latest_frame(self):
        return None


class FakePerception:
    """8 Hz 'detector': renders people as rectangles, runs the REAL PersonTracker, attaches ground-truth 3D."""

    def __init__(self, cfg, world):
        self.world, self.cfg = world, cfg
        self.tracker = PersonTracker(cfg["exp"]["tracker"])
        self.snapshot, self.crop, self.hz, self.error = None, None, 8.0, None
        self.run = True
        self.assign = {}                        # track id -> person name (ground truth, for the evaluation only)
        threading.Thread(target=self._loop, daemon=True).start()

    def set_want(self, **kw):
        pass

    def latest(self, max_age=None):
        return self.snapshot

    def _loop(self):
        while self.run:
            time.sleep(0.125)
            now = time.monotonic()
            img = np.zeros((480, 640, 3), np.uint8)
            dets, truth = [], []
            for name in self.world.people:
                b = self.world.person_in_base(name)
                if b is None or b[0] < 0.5 or b[0] > 6 or abs(math.atan2(b[1], b[0])) > 0.55:
                    continue
                x = b[0] - 0.09
                u = 320 - 480.7 * b[1] / x
                w, h = 480.7 * 0.5 / x, 480.7 * 1.7 / x
                box = np.array([u - w / 2, 240 - 0.45 * h, u + w / 2, 240 + 0.55 * h])
                dets.append((box, 0.9)); truth.append((name, box, b))
            order = sorted(range(len(dets)), key=lambda i: -truth[i][2][0])          # far first, near ones overdraw
            for i in order:
                x0, y0, x1, y1 = [int(max(0, v)) for v in dets[i][0]]
                img[y0:y1, x0:min(640, x1)] = self.world.people[truth[i][0]]["color"]
            tracks = self.tracker.update(dets, img, now)
            for tr in tracks:
                if tr.misses:
                    continue
                best = max(truth, key=lambda t: box_iou(tr.box, t[1]), default=None)
                if best is None:
                    continue
                self.assign[tr.id] = best[0]
                b = best[2] + np.random.normal(0, 0.01, 2)
                tr.xyz_base = np.array([b[0], b[1], 1.0])
                tr.dist, tr.bearing, tr.mono = float(np.hypot(*b)), float(math.atan2(b[1], b[0])), False
            fr = type("F", (), {"rgb": img})()
            self.snapshot = Snapshot(now, fr, np.eye(4), people=list(tracks), processed=now)


def run(name, people, seconds, params=None):
    cfg = config.load()
    cfg["exp"]["paths"]["results"] = tempfile.mkdtemp()
    world = World(people)
    io = FakeIO(cfg, world)
    perc = FakePerception(cfg, world)
    exp = FollowExperiment(Context(io, cfg, perc, ResultStore(cfg), lambda *_: None))
    exp.start(params or {})
    t0 = time.monotonic()
    rows = []
    while time.monotonic() - t0 < seconds:
        time.sleep(0.25)
        r = exp.runner
        tgt = r.target
        who = perc.assign.get(r.lock.id)
        truth = {n: world.person_in_base(n) for n in people}
        rows.append((world.t(), r.state, who, None if tgt is None else tgt.dist, world.vel.copy(), {n: (None if v is None else float(np.hypot(*v))) for n, v in truth.items()}))
    exp.stop()
    time.sleep(0.6)
    world.run = False
    perc.run = False
    return exp, world, rows


def show(name, exp, rows):
    print("== %s: losses %d, reacquired %d, rejected %d, final state %s, target person %s" % (
        name, exp.runner.losses, exp.runner.lock.reacquired, exp.runner.lock.rejected, exp.runner.state,
        rows[-1][2]))
    for r in rows[::8]:
        print("  t=%5.1f state=%-9s target=%-2s dist=%s speed=%.2f true=%s" % (r[0], r[1], r[2], None if r[3] is None else round(r[3], 2),
                                                                         np.hypot(r[4][0], r[4][1]), {k: None if v is None else round(v, 2) for k, v in r[5].items()}))


def scenario_straight():
    red = (200, 40, 40)
    return dict(A=dict(color=red, pos=lambda t: (2.2 + 0.25 * min(t, 16), 0.0) if t < 22 else (2.2 + 0.25 * 16, 0.0)))


def scenario_cross():
    return dict(A=dict(color=(200, 40, 40), pos=lambda t: (2.4 + 0.1 * t, 0.0)),
                B=dict(color=(40, 40, 200), pos=lambda t: (1.6 + 0.0 * t, 1.2 - 0.3 * (t - 6)) if 6 <= t <= 14 else (None if t < 6 else (1.6, -1.2))))


def scenario_occlusion():
    return dict(A=dict(color=(200, 40, 40), pos=lambda t: None if 9 <= t < 12.5 else (2.6 + 0.05 * min(t, 9), 0.0 if t < 9 else 0.35)))


def scenario_gone():
    return dict(A=dict(color=(200, 40, 40), pos=lambda t: (2.4, 0.0) if t < 8 else None))


def main():
    names = sys.argv[1:] or ["straight", "cross", "occlusion", "gone"]
    ok = True
    if "straight" in names:
        exp, w, rows = run("straight", scenario_straight(), 26)
        show("straight walk then stop", exp, rows)
        final = [r[3] for r in rows[-12:] if r[3]]
        good = bool(final) and abs(np.mean(final) - 1.2) < 0.3 and exp.runner.losses == 0
        print("   -> %s (final distance %.2f)" % ("PASS" if good else "FAIL", np.mean(final) if final else -1)); ok &= good
    if "cross" in names:
        exp, w, rows = run("cross", scenario_cross(), 24)
        show("second person crosses in front", exp, rows)
        wrong = [r for r in rows if r[2] == "B" and r[1] in ("following", "holding")]
        good = not wrong and rows[-1][2] == "A"
        print("   -> %s (frames following the wrong person: %d)" % ("PASS" if good else "FAIL", len(wrong))); ok &= good
    if "occlusion" in names:
        exp, w, rows = run("occlusion", scenario_occlusion(), 24)
        show("target hidden for 3.5 s, reappears 0.35 m to the side", exp, rows)
        good = exp.runner.losses >= 1 and rows[-1][1] in ("following", "holding") and rows[-1][2] == "A"
        print("   -> %s" % ("PASS" if good else "FAIL")); ok &= good
    if "gone" in names:
        exp, w, rows = run("gone", scenario_gone(), 30)
        show("target disappears for good", exp, rows)
        speeds = [np.hypot(r[4][0], r[4][1]) for r in rows[-8:]]
        good = exp.runner.losses >= 1 and max(speeds) < 0.02 and any("не найдена" in (exp.message or "") or exp.runner.state in ("idle", "locking", "gave_up") for _ in [0])
        print("   -> %s (state %s, message %r)" % ("PASS" if good else "FAIL", exp.runner.state, exp.message)); ok &= good
    print("ALL PASS" if ok else "SOME FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
