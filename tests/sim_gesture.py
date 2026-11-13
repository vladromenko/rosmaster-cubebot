"""Real-time simulation of GestureExperiment: scripted hand landmarks -> real analyzer -> real reactions on a simulated base.

Run:  python tests/sim_gesture.py        (about 60 s)
Checks: each gesture makes the correct physical motion (measured by odometry), the trial is judged from the motion,
holding a gesture fires one command, and an un-announced gesture is logged as an unarmed event.
"""
import os
import sys
import tempfile
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)
from cubebot import config  # noqa: E402
from cubebot.exp.common import Context, ResultStore, Snapshot  # noqa: E402
from cubebot.exp.gesture import GestureExperiment  # noqa: E402
from sim_follow import FakeIO, FakePerception, World  # noqa: E402
from test_exp import make_hand  # noqa: E402


class HandPerception(FakePerception):
    """No people; the 'hand' is whatever `self.script(t)` returns for the current time (a dict from make_hand or None)."""

    def __init__(self, cfg, world):
        self.script = lambda t: None
        super().__init__(cfg, world)

    def _loop(self):
        n = 0
        while self.run:
            time.sleep(0.125)
            now = time.monotonic()
            n += 1
            h = self.script(self.world.t())
            fr = type("F", (), {"rgb": np.zeros((480, 640, 3), np.uint8)})()
            self.snapshot = Snapshot(now + n * 1e-6, fr, np.eye(4), people=[], hands=[] if h is None else [h], processed=now)


def hold(kind, seconds, **kw):
    return lambda t: make_hand(kind, **kw) if t < seconds else None


def beckon(t):
    ph = (t % 0.85) / 0.85
    return make_hand("palm") if ph < 0.45 else make_hand("other")


def push(t):
    return make_hand("palm", scale=50 + min(t, 1.0) * 30)


def main():
    cfg = config.load()
    cfg["exp"]["paths"]["results"] = tempfile.mkdtemp()
    world = World({})
    io = FakeIO(cfg, world)
    perc = HandPerception(cfg, world)
    store = ResultStore(cfg)
    exp = GestureExperiment(Context(io, cfg, perc, store, lambda *_: None))
    exp.start({"execute": True, "follow": False})
    time.sleep(2.0)
    cases = [("stop", hold("palm", 2.5)), ("come", beckon), ("away", push), ("left", hold("point", 2.5, pdir=(-1, 0.1))),
             ("right", hold("point", 2.5, pdir=(1, 0.1))), ("resume", hold("thumb", 2.5))]
    ok = True
    for name, fn in cases:
        t0 = world.t()
        perc.script = lambda t, fn=fn, t0=t0: fn(t - t0) if t - t0 < 6.0 else None
        exp.command("trial_start", expect=name, distance_m=1.5, background="sim", light="sim")
        deadline = time.monotonic() + 14
        while exp.trial is not None and time.monotonic() < deadline:
            time.sleep(0.2)
        perc.script = lambda t: None
        time.sleep(2.0)                                     # hand released: latch opens, cooldown passes
        rows = [r for r in store.load(exp.key) if r.get("kind") == "trial"]
        r = rows[-1] if rows else None
        good = r is not None and r["expect"] == name and r["correct"]
        print("%-7s -> got %-8s motion_ok=%s dx=%s dyaw=%s %s" % (name, None if r is None else r["got"], None if r is None else r["motion_ok"],
                                                               None if r is None else r["dx"], None if r is None else r["dyaw_deg"], "PASS" if good else "FAIL"))
        ok &= good
        events_in_trial = None if r is None else r["extra_events"]
        if events_in_trial:
            print("   extra events during one held gesture:", events_in_trial)
            ok = False
    # an un-announced gesture must be recorded as unarmed, not as a trial
    t0 = world.t()
    perc.script = lambda t: hold("point", 2.5, pdir=(-1, 0.1))(t - t0)
    time.sleep(5)
    perc.script = lambda t: None
    un = [r for r in store.load(exp.key) if r.get("kind") == "unarmed_event"]
    print("unarmed events logged:", [(r["gesture"], r["dyaw_deg"]) for r in un], "PASS" if un and un[-1]["gesture"] == "left" else "FAIL")
    ok &= bool(un)
    exp.stop()
    time.sleep(0.5)
    world.run = False
    perc.run = False
    print("ALL PASS" if ok else "SOME FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
