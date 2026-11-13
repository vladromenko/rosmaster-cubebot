"""Owns the five experiments inside the console: one at a time, shared perception thread, shared navigation stack."""
import threading
import time

from .assistant import AssistantExperiment
from .common import Context, Perception, ResultStore
from .follow import FollowExperiment, FollowRunner
from .gesture import GestureAnalyzer, GestureExperiment
from .navigation import NavExperiment, NavStack
from .retrieve import RetrieveExperiment


class ExperimentManager:
    ORDER = ("e1_follow", "e2_gesture", "e3_nav", "e4_retrieve", "e5_assistant")

    def __init__(self, console):
        self.console = console
        self.io, self.cfg, self.log = console.io, console.cfg, console.log
        self.store = ResultStore(self.cfg)
        self.perception = Perception(self.io, self.cfg, self.log)
        self._nav = None
        ctx = Context(self.io, self.cfg, self.perception, self.store, self.log, console)
        self.exps = {
            "e1_follow": FollowExperiment(ctx),
            "e2_gesture": GestureExperiment(ctx),
            "e3_nav": NavExperiment(ctx, self.nav),
            "e4_retrieve": RetrieveExperiment(ctx, self.nav),
            "e5_assistant": AssistantExperiment(ctx, self.nav),
        }
        self.active = None
        self.error = None
        # Preview: the real perception + follow controller + gesture analyzer run on the camera WITHOUT moving the robot,
        # so the docs page can show live data without starting an experiment.
        self.preview_want = dict(people=False, hands=False, pose=False)
        self._pv_runner = FollowRunner(ctx)
        self._pv_analyzer = GestureAnalyzer(self.cfg["exp"]["gesture"])
        self._pv_stamp = None
        self._pv_run = True
        threading.Thread(target=self._preview_loop, daemon=True).start()

    # ---- shared navigation stack
    def nav(self, create=True):
        if self._nav is None and create:
            self._nav = NavStack(self.io, self.cfg, self.log)
        return self._nav

    def stop_nav(self):
        if self._nav is not None and self._nav.running:
            self._nav.stop()

    # ---- preview (no motion)
    def preview(self, people=None, hands=None, pose=None):
        for k, v in (("people", people), ("hands", hands), ("pose", pose)):
            if v is not None:
                self.preview_want[k] = bool(v)
        if not any(self.preview_want.values()) and not (self.active is not None and self.active.running):
            self.perception.set_want(people=False, hands=False, pose=False, objects=False)
            self.perception.crop = None
        elif self.active is None or not self.active.running:
            self._pv_runner.unlock()

    def follow_runner(self):
        a = self.active
        if a is not None and a.running and getattr(a, "runner", None) is not None:
            return a.runner
        return self._pv_runner if self.preview_want["people"] else None

    def analyzer(self):
        a = self.active
        if a is not None and a.running and a.key == "e2_gesture":
            return a.analyzer
        return self._pv_analyzer if self.preview_want["hands"] else None

    def _preview_loop(self):
        while self._pv_run:
            time.sleep(0.08)
            if self.active is not None and self.active.running:
                continue
            w = self.preview_want
            if not any(w.values()):
                continue
            p = self.perception
            p.set_want(people=w["people"] or w["hands"], hands=w["hands"], pose=w["pose"], objects=False)
            snap = p.latest(max_age=1.0)
            if snap is None or snap.stamp == self._pv_stamp:
                continue
            self._pv_stamp = snap.stamp
            now = time.monotonic()
            try:
                if w["hands"]:
                    people = [t for t in snap.people if t.misses == 0 and t.dist is not None]
                    if people:
                        t = min(people, key=lambda x: x.dist)
                        x0, y0, x1, y1 = t.box
                        p.crop = (x0 - 0.9 * (x1 - x0), y0 - 0.1 * (y1 - y0), x1 + 0.9 * (x1 - x0), y0 + 0.75 * (y1 - y0))
                    else:
                        p.crop = None
                    self._pv_analyzer.update(snap.hands, now)
                if w["people"]:
                    self._pv_runner.tick(snap, now, allow_motion=False)
            except Exception as exc:
                self.error = "preview: %r" % (exc,)

    # ---- run control
    def start(self, key, params=None):
        con = self.console
        exp = self.exps.get(key)
        if exp is None:
            return self._err("unknown experiment %r" % key)
        if self.active is not None and self.active.running:
            return self._err("experiment %s is still running: stop it first" % self.active.key)
        if con.mode != "manual":
            return self._err("console is busy (%s)" % con.mode)
        if con.battery_low():
            return self._err("battery too low")
        self.error = None
        self.preview_want = dict(people=False, hands=False, pose=False)
        for section, values in (params or {}).get("cfg", {}).items():       # live tuning from the UI
            if section in self.cfg["exp"] and isinstance(values, dict):
                self.cfg["exp"][section].update(values)
        con.mode = "exp"
        self.active = exp
        exp.metrics = {}
        exp.state, exp.message = "starting", ""
        exp.start(params)
        threading.Thread(target=self._watch, args=(exp,), daemon=True).start()
        return True

    def _watch(self, exp):
        while exp.running:
            time.sleep(0.2)
        self.perception.set_want(people=False, objects=False, hands=False, pose=False)
        self.perception.crop = None
        self.io.stop_base()
        self.console.jog.reset(self.io.arm_command())
        self.console.mode = "manual"

    def stop(self, reason="operator stop"):
        if self.active is not None and self.active.running:
            self.active.stop(reason)
        else:
            self.io.stop_base()

    def command(self, key, name, kw=None):
        exp = self.exps.get(key)
        if exp is None or not hasattr(exp, "command"):
            return self._err("no such command target")
        try:
            return exp.command(name, **(kw or {}))
        except Exception as exc:
            return self._err("command %s failed: %r" % (name, exc))

    def _err(self, text):
        self.error = text
        self.log("experiments: %s" % text)
        return False

    # ---- video
    def render(self, bgr):
        exp = self.active
        if exp is not None and exp.running:
            try:
                bgr = exp.overlay(bgr)
                exp.record(bgr)
            except Exception as exc:                        # an overlay bug must never stop the video
                self.error = "overlay: %r" % (exc,)
        return bgr

    # ---- state for the UI
    def state(self):
        nav = self._nav
        return dict(
            active=None if self.active is None or not self.active.running else self.active.key,
            error=self.error, perception=dict(hz=round(self.perception.hz, 1), error=self.perception.error),
            exps={k: e.status() for k, e in self.exps.items()},
            nav=dict(running=bool(nav and nav.running), waypoints={} if nav is None else nav.waypoints,
                     pose=None if nav is None or not nav.running else nav.pose_map(),
                     map=bool(nav and nav.map is not None)))

    def shutdown(self):
        try:
            self.stop("console shutdown")
        finally:
            self._pv_run = False
            self.perception.shutdown()
            if self._nav is not None and self._nav.running:
                self._nav.stop()
