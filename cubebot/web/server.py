"""Rosmaster console: browser UI with camera, smooth manual control (keyboard/gamepad), battery and CubeBot missions.

Run on the robot: scripts/console.sh [--dry-run]   then open http://explorer.local:8090 on the Mac.
The console owns /cmd_vel and /arm6_joints while it runs (like CubeBot), so the launcher pauses Explorer's
control/watchdog services and restores them afterwards.
"""
import argparse
import asyncio
import json
import threading
import time
from collections import deque

import cv2
import numpy as np
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, StreamingResponse

from .. import config
from ..arm_control import CartesianJog
from ..arm_model import ArmModel
from ..battery import BatteryGauge
from .. import mirror as mirror_mod
from ..mission import Mission
from ..robot_io import MissionAbort
from ..vision import detect_cubes
from ..exp.manager import ExperimentManager
from .page import INDEX_HTML


class Console:
    CONTROL_HZ = 20.0

    def __init__(self, dry_run, arm_now, port_cfg=None):
        from ..ros_io import RosRobot
        self.cfg = config.load(port_cfg)
        self.arm = ArmModel(self.cfg["arm"]["tcp_offset_m"], optical_rotation=self.cfg["camera"]["optical_rotation"])
        self.lines = deque(maxlen=60)
        self.dry_run = dry_run
        self.io = RosRobot(self.cfg, self.arm, log=self.log, node_name="rosmaster_console", dry_run=dry_run)
        self.io._arm_cmd = [float(v) for v in arm_now]
        self.jog = CartesianJog(self.arm, self.cfg, self.io.arm_command())
        self.mirror_cfg = mirror_mod.load()
        self.mirror = mirror_mod.MirrorMapper(self.arm, self.cfg["arm"], self.mirror_cfg, self.io.arm_command())
        self.mirror_count = 0
        self.gauge = BatteryGauge()
        self.arm_mode = "cartesian"
        self._arm_input_active = False
        self._arm_tracking_blocked = False
        self.mode = "manual"
        self.mission_state = "idle"
        self.mission_result = None
        self.last_input = 0.0
        self.input = dict(drive=[0, 0, 0], arm=[0, 0, 0], pitch=0, roll=0, grip=0, slow=False)
        self.busy_arm = False
        self.resume_with_cube = False
        self._lock = threading.RLock()
        self._jpeg, self._jpeg_stamp = None, 0.0
        self._running = True
        self.exp = ExperimentManager(self)
        for fn in (self._control_loop, self._battery_loop, self._video_loop):
            threading.Thread(target=fn, daemon=True).start()
        threading.Thread(target=self._sync_arm, daemon=True).start()
        self.log("console ready (%s)" % ("DRY-RUN: no motion is sent" if dry_run else "live"))

    def _sync_arm(self):
        """Take the real arm pose from the servo feedback as soon as the bridge delivers it."""
        if self.io.sync_arm_estimate(wait=20.0):
            self.jog.reset(self.io.arm_command())
            self.mirror.servo = np.array(self.io.arm_command(), float)
            self.mirror.target = self.mirror.servo.copy()

    def log(self, text):
        line = "%s %s" % (time.strftime("%H:%M:%S"), text)
        self.lines.append(line)
        print(line, flush=True)

    # --- operator input ----------------------------------------------------------------
    def set_input(self, msg):
        self.last_input = time.monotonic()
        for key in ("drive", "arm", "pitch", "roll", "grip", "slow"):
            if key in msg:
                self.input[key] = msg[key]
        if "free" in msg and self.mode == "manual":
            self.io.guard_off = bool(msg["free"])

    def stop_all(self):
        with self._lock:
            self._stop_locked()

    def _stop_locked(self):
        self.input.update(drive=[0, 0, 0], arm=[0, 0, 0], pitch=0, roll=0, grip=0)
        self.jog.velocity[:] = 0
        self._arm_input_active = False
        self._arm_tracking_blocked = False
        if self.mode == "mirror":
            self.mode = "manual"
            self.mirror.rebase()
        self.io.stop_base()
        if self.mode == "mission":
            self.io.request_abort("operator stop")
        if self.mode == "exp":
            self.exp.stop()
        self.log("STOP")

    def action(self, msg):
        name = msg.get("action")
        if name == "stop":
            self.stop_all()
        elif name == "arm_mode" and self.mode == "manual" and not self.busy_arm:
            mode = msg.get("mode")
            if mode not in ("cartesian", "joint"):
                raise ValueError("Unknown arm mode")
            self.stop_all()
            self.arm_mode = mode
            self.jog.reset(self.io.measured_arm(max_age=.25) or self.io.arm_command())
            self.log("arm mode: " + mode)
        elif name == "preset" and self.mode == "manual" and not self.busy_arm:
            self._run_background(self._preset, msg.get("name"))
        elif name == "mirror" and self.mode in ("manual", "mirror"):
            on = bool(msg.get("on"))
            if on:
                if self.busy_arm or self.io.measured_arm(max_age=.25) is None:
                    self.log("mirror refused: arm busy or feedback stale")
                    return
                if time.monotonic() - self.mirror.last_msg > self.mirror.m["timeout_s"]:
                    self.log("mirror refused: start the Leader stream first")
                    return
                self.mirror.servo = np.array(self.io.measured_arm() or self.io.arm_command(), float)
                self.mirror.target = self.mirror.servo.copy()
                self.mirror.rebase()
                self.io._arm_cmd = [float(v) for v in self.mirror.servo]
                self.mode = "mirror"
                self.log("mirror ON: the leader arm now drives the arm (small motions first!)")
            else:
                self.mode = "manual"
                self.jog.reset(self.io.arm_command())
                self.log("mirror OFF")
        elif name == "exp_start":
            self.exp.start(msg.get("key"), msg.get("params") or {})
        elif name == "exp_preview":
            if self.mode == "manual":
                self.exp.preview(msg.get("people"), msg.get("hands"), msg.get("pose"))
        elif name == "exp_stop":
            self.exp.stop()
        elif name == "exp_cmd":
            self.exp.command(msg.get("key"), msg.get("cmd"), msg.get("args") or {})
        elif name == "exp_nav_stop":
            self.exp.stop_nav()
        elif name == "mission_start":
            self.start_mission()
        elif name == "mission_resume":
            self.resume_with_cube = True              # the cube is already in the gripper: continue with the box
            self.start_mission()
        elif name == "debug_box":
            self._run_background(self._debug_box, msg.get("pose", "near"))
        elif name == "mission_stop":
            self.io.request_abort("operator stop")
        elif name == "arm_assume":
            self.io._arm_cmd = [float(v) for v in msg["servo"]]
            self.jog.reset(self.io.arm_command())
            self.log("arm estimate set to %s" % self.io.arm_command())

    def _run_background(self, fn, *args):
        threading.Thread(target=fn, args=args, daemon=True).start()

    def _preset(self, name):
        a = self.cfg["arm"]
        pose = {"look": a["search_pose"], "near": a["near_search_pose"], "carry": a["carry_pose"],
                "home": a["safe_pose"], "people": self.cfg["exp"]["people_pose"]}.get(name)
        if not pose:
            return
        self.busy_arm = True
        try:
            self.io.abort_reason = None
            self.log("arm -> %s" % name)
            self.io.move_arm(list(pose))
            self.jog.reset(self.io.arm_command())
        except Exception as exc:
            self.log("preset failed: %s" % exc)
        finally:
            self.busy_arm = False

    def _debug_box(self, which):
        """Diagnostics: look at the box from the current spot with both detectors and save the annotated frame."""
        import cv2
        from ..vision import detect_box, detect_box_by_color, learn_box_color
        try:
            m = Mission(self.io, self.cfg, self.log)
            pose = list(self.cfg["arm"]["near_search_pose"] if which == "near" else self.cfg["arm"]["search_pose"])
            pose[5] = int(round(self.io.arm_command()[5]))
            self.io.move_arm(pose)
            look = m.look(settle=0.5)
            box = detect_box(look.frame, look.t_base_cam, 0.0, m.box_cfg, m.cam_cfg)
            self.log("debug box (%s view): geometry %s" % (which, None if box is None else
                     "centre %s rim %s" % (box.center_xy.round(3), box.near_xy.round(3))))
            model = learn_box_color(look.frame, look.t_base_cam, box) if box is not None else None
            self.log("debug box: colour model %s" % (model,))
            img = cv2.cvtColor(look.frame.rgb, cv2.COLOR_RGB2BGR)
            if model is not None:
                cbox = detect_box_by_color(look.frame, look.t_base_cam, model)
                self.log("debug box: colour detection %s" % (None if cbox is None else "rim %s" % cbox.near_xy.round(3)))
                mask = cv2.inRange(cv2.cvtColor(look.frame.rgb, cv2.COLOR_RGB2HSV), np.array(model["lo"], np.uint8),
                                   np.array(model["hi"], np.uint8))
                img[mask > 0] = (0, 0, 255)
            cv2.imwrite("/tmp/debug_box.png", img)
        except Exception as exc:
            self.log("debug box failed: %r" % (exc,))

    # --- missions --------------------------------------------------------------------------
    def start_mission(self):
        if self.mode == "mission":
            return
        if self.battery_low():
            self.log("mission refused: battery too low")
            return
        self.mode = "mission"
        self.mission_state, self.mission_result = "running", None
        self._run_background(self._mission_thread)

    def battery_low(self):
        s = self.gauge.state()
        return s["volts"] is not None and s["volts"] < self.cfg["mission"]["min_battery_v"] + 0.3

    def _mission_thread(self):
        io = self.io
        try:
            io.abort_reason, io.started = None, None
            mission = Mission(io, self.cfg, self.log)
            if self.resume_with_cube:
                self.resume_with_cube = False
                io.sync_arm_estimate()
                mission.resume_with_cube()
            ok = mission.run()
            self.mission_result = mission.result
            self.mission_state = "done" if ok else "failed"
        except MissionAbort as exc:
            self.mission_result, self.mission_state = "aborted: %s" % exc, "failed"
        except Exception as exc:
            self.mission_result, self.mission_state = "error: %r" % (exc,), "failed"
        finally:
            io.abort_reason = None
            io.stop_base()
            self.jog.reset(io.arm_command())
            self.mode = "manual"

    # --- loops -----------------------------------------------------------------------------
    def _control_loop(self):
        period = 1.0 / self.CONTROL_HZ
        last = time.monotonic()
        while self._running:
            time.sleep(max(0.0, period - (time.monotonic() - last)))
            now = time.monotonic()
            dt, last = now - last, now
            try:
                if self.mode == "mirror":
                    with self._lock:
                        if self.mode == "mirror":
                            self._mirror_step(dt, now)
                    continue
                if self.mode != "manual" or self.busy_arm:
                    continue
                with self._lock:
                    if self.mode == "manual" and not self.busy_arm:
                        self._manual_step(dt, now)
            except MissionAbort:
                pass
            except Exception as exc:
                self.log("control error: %r" % (exc,))
                self.io.stop_base()

    def _mirror_step(self, dt, now):
        measured = self.io.measured_arm(max_age=.25)
        tracking = measured is not None and max(abs(a-b) for a,b in
            zip(self.io.arm_command()[:5], measured[:5])) < 12.
        if now - self.mirror.last_msg > self.mirror.m["timeout_s"] or not 0 < dt <= .15 or not tracking:
            self.stop_all()
            self.mirror.blocked = "stream/feedback lost; enable mirror again after recovery"
            self.log(self.mirror.blocked)
            return
        self.io.drive(0., 0., 0.)
        cmd = self.mirror.step(dt, now)
        if cmd:
            self.io.move_arm(cmd, duration_ms=110, wait=False, check=True)

    def _manual_step(self, dt, now):
        fresh = now - self.last_input < 0.4
        x = self.input
        if not fresh:
            x = dict(drive=[0, 0, 0], arm=[0, 0, 0], pitch=0, roll=0, grip=0, slow=False)
            self.jog.velocity[:] = 0
        speed = 0.4 if x["slow"] else 1.0
        b = self.cfg["manual_base"]
        self.io.drive(x["drive"][0] * b["max_vx"] * speed, x["drive"][1] * b["max_vy"] * speed,
                      x["drive"][2] * b["max_wz"] * speed, manual=True)
        active = any(abs(v) > .001 for v in [*x["arm"], x["pitch"], x["roll"], x["grip"]])
        measured = self.io.measured_arm(max_age=.25)
        if active and not self._arm_input_active:
            if measured is None:
                self.jog.blocked = "no fresh arm feedback"
            else:
                self.io._arm_cmd = list(measured)
                self.jog.reset(measured)
        tracking_ok = measured is not None and max(abs(a-b) for a,b in
            zip(self.io.arm_command()[:5], measured[:5])) < 12.
        if not active:
            self._arm_tracking_blocked = False
            self.jog.velocity[:] = 0
        if active and (not tracking_ok or not 0 < dt <= .15):
            self._arm_tracking_blocked = True
        if self._arm_tracking_blocked:
            self.jog.blocked = "arm tracking error; release controls"
            self.jog.velocity[:] = 0
        elif active:
            if self.arm_mode == "joint":
                rates = [-x["arm"][1], x["arm"][0], x["arm"][2], x["pitch"], x["roll"], x["grip"]]
                cmd = self.jog.step_joints(dt, rates, x["slow"])
            else:
                cmd = self.jog.step(dt, x["arm"], x["pitch"], x["roll"], x["grip"], x["slow"])
            if cmd:
                self.io.move_arm(cmd, duration_ms=self.jog.run_ms, wait=False, check=True)
        self._arm_input_active = active

    def _battery_loop(self):
        while self._running:
            moving = any(abs(v) > 0.01 for v in self.io.last_output) or self.mode == "mission"
            arm_moving = self.io.now() < self.io._arm_busy_until
            self.gauge.update(self.io.battery(), base_moving=moving, arm_moving=arm_moving)
            time.sleep(1.0)

    def _video_loop(self):
        n = 0
        while self._running:
            frame = self.io._latest_frame()
            if frame is None or frame.stamp == self._jpeg_stamp:
                time.sleep(0.03)
                continue
            img = cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2BGR)
            n += 1
            if self.mode == "exp":
                img = self.exp.render(img)
            elif n % 3 == 0:
                try:
                    self._marks = detect_cubes(frame, self.cfg["cube"], self.cfg["camera"])
                except Exception:
                    self._marks = []
            for obs in (getattr(self, "_marks", []) if self.mode != "exp" else []):
                cv2.circle(img, (int(obs.u), int(obs.v)), 16, (0, 0, 255), 2)
                cv2.putText(img, "cube %.2f m" % obs.range_m, (int(obs.u) + 18, int(obs.v)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, (0, 0, 255), 2)
            ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                self._jpeg, self._jpeg_stamp = buf.tobytes(), frame.stamp
            time.sleep(0.08)

    def telemetry(self):
        io = self.io
        tcp = self.arm.tcp_position(io.arm_command())
        return dict(battery=self.gauge.state(), battery_stale=self.gauge.stale(moving=self.mode == "mission"),
                    pose=[round(v, 3) for v in io.pose()], arm=[round(v) for v in io.arm_command()],
                    arm_measured=None if io.measured_arm() is None else [round(v) for v in io.measured_arm()],
                    mirror=dict(on=self.mode == "mirror", blocked=self.mirror.blocked, frames=self.mirror_count,
                                age=round(time.monotonic() - self.mirror.last_msg, 2) if self.mirror.last_msg > 0 else None,
                                target=[round(float(v)) for v in self.mirror.target]),
                    tcp=[round(float(v), 3) for v in tcp], mode=self.mode, arm_mode=self.arm_mode, manual_base=self.cfg["manual_base"], mission=self.mission_state,
                    mission_result=self.mission_result, guard=io.guard.last_block, cmd=[round(v, 3) for v in io.last_output], guard_off=io.guard_off, input=self.input, jog_blocked=self.jog.blocked,
                    sensors=io.sensors_alive(), dry_run=self.dry_run, odom=io.odometry.stats, log=list(self.lines)[-14:],
                    exp=self.exp.state())

    def shutdown(self):
        self._running = False
        try:
            self.exp.shutdown()
        finally:
            self.io.shutdown()


def _jsonable(o):
    if hasattr(o, "tolist"):
        return o.tolist()
    if hasattr(o, "item"):
        return o.item()
    return str(o)


def build_app(console):
    app = FastAPI()
    from .live import register as register_live
    register_live(app, console)

    @app.get("/", response_class=HTMLResponse)
    def index():
        return INDEX_HTML

    @app.get("/tutorial", response_class=HTMLResponse)
    def tutorial():
        import pathlib
        return (pathlib.Path(__file__).parent / "tutorial.html").read_text(encoding="utf-8")

    @app.get("/experiments", response_class=HTMLResponse)
    def experiments_page():
        import pathlib
        return (pathlib.Path(__file__).parent / "experiments.html").read_text(encoding="utf-8")

    @app.get("/exp-docs", response_class=HTMLResponse)
    def experiments_docs():
        import pathlib
        return (pathlib.Path(__file__).parent / "exp_docs.html").read_text(encoding="utf-8")

    @app.get("/api/exp/results/{key}")
    def exp_results(key: str):
        return console.exp.store.load(key, limit=300)

    @app.get("/api/exp/map.png")
    def exp_map():
        from fastapi.responses import Response
        nav = console.exp.nav(create=False)
        if nav is None:
            import cv2
            img = np.full((100, 300, 3), 40, np.uint8)
            cv2.putText(img, "navigation not started", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            return Response(cv2.imencode(".png", img)[1].tobytes(), media_type="image/png")
        return Response(nav.map_png(), media_type="image/png")

    @app.get("/video")
    def video():
        def gen():
            last = None
            while console._running:
                jpg = console._jpeg
                if jpg is not None and jpg is not last:
                    last = jpg
                    yield b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpg) + jpg + b"\r\n"
                time.sleep(0.05)
        return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame")

    @app.get("/api/state")
    def state():
        return console.telemetry()

    @app.websocket("/ws")
    async def ws(socket: WebSocket):
        await socket.accept()

        async def push():
            while True:
                await socket.send_text(json.dumps(console.telemetry(), default=_jsonable))
                await asyncio.sleep(0.1)

        task = asyncio.create_task(push())
        try:
            while True:
                msg = json.loads(await socket.receive_text())
                if "action" in msg:
                    console.action(msg)
                else:
                    console.set_input(msg)
        except (WebSocketDisconnect, RuntimeError):
            pass
        finally:
            task.cancel()
            console.input.update(drive=[0, 0, 0], arm=[0, 0, 0], pitch=0, roll=0, grip=0)
            console.last_input = 0.0
            console.stop_all()

    # Leader stream from Homebrain (Explorer service, LeRobot read-only), started from this console.
    HOMEBRAIN = "http://192.168.0.168:8030"

    def _homebrain(path):
        import urllib.request
        req = urllib.request.Request(HOMEBRAIN + path, data=b"", method="POST")
        return urllib.request.urlopen(req, timeout=5).read()

    @app.post("/api/mirror_on")
    async def mirror_on():
        if console.mode == "mirror":
            return {"ok": True}
        try:
            await asyncio.to_thread(_homebrain, "/api/rosmaster/leader8090/start")
        except Exception as exc:
            console.log("Leader: Homebrain недоступен: %s" % exc)
            return {"ok": False, "error": str(exc)}
        end = time.monotonic() + 6
        while time.monotonic() < end and time.monotonic() - console.mirror.last_msg > 0.3:
            await asyncio.sleep(0.1)
        console.action({"action": "mirror", "on": True})
        return {"ok": console.mode == "mirror"}

    @app.post("/api/mirror_off")
    async def mirror_off():
        console.action({"action": "mirror", "on": False})
        try:
            await asyncio.to_thread(_homebrain, "/api/rosmaster/leader8090/stop")
        except Exception as exc:
            console.log("Leader: Homebrain недоступен: %s" % exc)
        return {"ok": True}

    @app.websocket("/mirror")
    async def mirror_ws(socket: WebSocket):
        await socket.accept()
        try:
            while True:
                msg = json.loads(await socket.receive_text())
                if console.mirror.update(msg.get("leader", {}), time.monotonic()):
                    console.mirror_count += 1
        except (WebSocketDisconnect, RuntimeError, ValueError):
            pass
        finally:
            if console.mode == "mirror":
                console.stop_all()

    return app


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--dry-run", action="store_true", help="never publish motion (UI and telemetry only)")
    ap.add_argument("--arm-now", default="90,90,90,90,90,90", help="current commanded arm pose (no servo feedback exists)")
    ap.add_argument("--config")
    args = ap.parse_args(argv)
    console = Console(args.dry_run, [float(v) for v in args.arm_now.split(",")], args.config)
    try:
        uvicorn.run(build_app(console), host="0.0.0.0", port=args.port, log_level="warning", timeout_graceful_shutdown=3)
    finally:
        console.shutdown()


if __name__ == "__main__":
    main()
