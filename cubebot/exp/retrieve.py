"""Experiment 4: find a named object, grasp it and deliver it to a given point.

The proven cube pipeline (`Mission`: search sweep, stop-and-go approach, camera-closed-loop grasp with IK and overdrive
squeeze, in-hand check) is reused unchanged. This module adds
  * a target resolver ("найди бутылку" -> COCO label),
  * a detector adapter so the same pipeline can chase other small objects found by the EfficientDet model,
  * a graspability check (does it fit between the open jaws?),
  * map-assisted search (visit the saved search points through Nav2 when the room sweep finds nothing),
  * delivery to a point (map waypoint through Nav2, or the odometry start pose) with a camera check afterwards,
  * a per-stage record: detected / reached / grasped / delivered.
"""
import math
import re
import time

import numpy as np

from .. import mission as mission_module
from ..geometry import se2_between, wrap
from ..mission import Mission, MissionFailed
from ..robot_io import MissionAbort
from ..vision import CubeObs, deproject
from .common import Experiment, base_limits, put_text, region_depth
from .motion import translate, turn_by


def resolve_target(text, ecfg):
    """Map a free-text command to 'cube' or a COCO label; None when nothing is recognised."""
    t = text.lower()
    for stem, label in ecfg["retrieve"]["labels_ru"].items():
        if stem in t:
            return label
    for w in re.findall(r"[a-z ]+", t):
        w = w.strip()
        if w in ("cube", "bottle", "cup", "apple", "banana", "orange", "sports ball", "teddy bear"):
            return w
    return None


class DetectorAdapter:
    """Looks like `detect_cubes(frame, cube_cfg, cam_cfg)` but finds a COCO object (own detector instance: the
    perception thread keeps its own, MediaPipe task objects are not thread safe)."""

    def __init__(self, perception, label, ecfg):
        self.label, self.ecfg = label, ecfg
        self.det = perception.new_detector()
        self.mp_image = perception._mp_image
        self.last_width_m = None

    def __call__(self, frame, cube_cfg, cam_cfg):
        out = []
        res = self.det.detect(self.mp_image(frame.rgb))
        K = frame.K
        for d in res.detections:
            c = d.categories[0]
            if c.category_name != self.label or c.score < self.ecfg["detector"]["object_score"]:
                continue
            bb = d.bounding_box
            x0, y0, w, h = bb.origin_x, bb.origin_y, bb.width, bb.height
            u, v = x0 + w / 2, y0 + h / 2
            z = region_depth(frame.depth, u - 0.2 * w, v - 0.2 * h, u + 0.2 * w, v + 0.2 * h, lo=cam_cfg["min_depth_m"],
                             hi=cam_cfg["max_depth_m"])
            if z is None:
                continue
            width_m, height_m = w * z / K[0, 0], h * z / K[1, 1]
            self.last_width_m = min(width_m, height_m)
            size = min(width_m, height_m)
            patch = frame.depth[int(y0):int(y0 + h), int(x0):int(x0 + w)]
            ys, xs = np.nonzero((patch > cam_cfg["min_depth_m"]) & (patch < cam_cfg["max_depth_m"]))
            pts = None
            if len(ys) > 8:
                pick = np.linspace(0, len(ys) - 1, min(len(ys), 150)).astype(int)
                zz = patch[ys[pick], xs[pick]]
                pts = np.stack([(xs[pick] + x0 - K[0, 2]) * zz / K[0, 0], (ys[pick] + y0 - K[1, 2]) * zz / K[1, 1], zz], 1)
            rng = z + 0.5 * size
            out.append(CubeObs(float(u), float(v), float(min(w, h)), rng, deproject(u, v, rng, K), True, float(c.score),
                               False, pts))
        out.sort(key=lambda o: -o.score)
        return out


class Retriever(Mission):
    def __init__(self, io, cfg, log, label, deliver, perception, nav=None):
        super().__init__(io, cfg, log)
        self.label, self.deliver, self.perception, self.nav = label, deliver, perception, nav
        self.rc = cfg["exp"]["retrieve"]
        self.stages = dict(detected=False, reached=False, grasped=False, delivered=False)
        self.fail_reason = None
        self.adapter = None
        self.start_pose = tuple(io.pose())
        self.t_stage = {}

    # ---- override: our own state graph
    def run(self):
        handlers = {"PREFLIGHT": self.s_preflight, "SEARCH_CUBE": self.s_search_cube, "APPROACH_CUBE": self.s_approach_cube,
                    "GRASP": self.s_grasp, "DELIVER": self.s_deliver, "VERIFY": self.s_verify}
        saved = mission_module.detect_cubes
        saved_size = self.cube_cfg["size_m"]
        if self.label != "cube":
            self.adapter = DetectorAdapter(self.perception, self.label, self.cfg["exp"])
            mission_module.detect_cubes = self.adapter
            self.cube_cfg["size_m"] = min(0.06, self.cube_cfg["size_m"])
        try:
            while self.state not in ("DONE", "FAILED"):
                self.say("enter")
                self.t_stage[self.state] = time.monotonic()
                self.state = handlers[self.state]()
        except MissionAbort as exc:
            self.result, self.fail_reason, self.state = "aborted: %s" % exc, "aborted", "FAILED"
        except MissionFailed as exc:
            self.result, self.fail_reason, self.state = "failed: %s" % exc, str(exc), "FAILED"
        finally:
            mission_module.detect_cubes = saved
            if self.label != "cube":
                self.cube_cfg["size_m"] = saved_size            # the guess for another object must not leak into the cube mission
            self.io.stop_base()
            if self.nav is not None:
                self.nav.drive_enabled = False
            try:
                self.io.abort_reason = None
                park = list(self.arm_cfg["safe_pose"])
                if self.deliver.get("kind") == "hold" and self.state == "DONE":
                    park = None                                         # the assistant continues with the object in hand
                elif not self.cube_released:
                    park[5] = int(round(self.io.arm_command()[5]))     # holding something: do not drop it while parking
                if park is not None:
                    self.io.move_arm(park, check=False, wait=False)
            except Exception as exc:
                self.log("arm park failed: %s" % exc)
        if self.state == "DONE":
            self.result = self.result or "delivered"
        self.log("retrieve result: %s" % self.result)
        return self.state == "DONE"

    # ---- search (room sweep, then saved search points through Nav2)
    def s_search_cube(self):
        try:
            nxt = super().s_search_cube()
            self.stages["detected"] = True
            return self.check_graspable(nxt)
        except MissionFailed as exc:
            if self.nav is None or not self.nav.running or not self.rc["map_search"]:
                raise
            self.say("sweep found nothing (%s): visiting the saved search points" % exc)
        for name, w in list(self.nav.waypoints.items()):
            if name in ("home", "drop"):
                continue
            pose = self.nav.pose_map()
            if pose and math.hypot(pose[0] - w["x"], pose[1] - w["y"]) < 0.7:
                continue
            self.say("search point %s" % name)
            self.nav_goto(w["x"], w["y"], w["yaw"], name)
            if self.sweep_for_cube(self.search_pose()) or self.sweep_for_cube(list(self.arm_cfg["near_search_pose"])):
                self.io.move_arm(self.search_pose())
                self.stages["detected"] = True
                return self.check_graspable("APPROACH_CUBE")
        raise MissionFailed("object not found: room sweep and %d search points" % len(self.nav.waypoints))

    def check_graspable(self, nxt):
        """Fits between the open jaws? (generic objects only; the cube size is measured by the mission itself)"""
        if self.adapter is None or self.adapter.last_width_m is None:
            return nxt
        width = self.adapter.last_width_m
        limit = min(self.rc["max_object_width_m"], self.arm_cfg["gripper_width_open_m"] - 0.015)
        self.say("object width %.0f mm, jaws open to %.0f mm" % (width * 1e3, limit * 1e3))
        if width > limit:
            raise MissionFailed("object not graspable: %.0f mm wide, the gripper opens to %.0f mm" % (width * 1e3, limit * 1e3))
        self.cube_cfg["size_m"] = float(min(max(width, 0.02), 0.08))
        return nxt

    def s_approach_cube(self):
        nxt = super().s_approach_cube()
        self.stages["reached"] = True
        return nxt

    def s_grasp(self):
        nxt = super().s_grasp()                       # returns "SEARCH_BOX" after a verified grasp, raises otherwise
        self.stages["grasped"] = True
        return "DELIVER"

    # ---- delivery
    def nav_goto(self, x, y, yaw, label):
        limits = self.cfg["exp"]["nav"]
        self.nav.drive_enabled = True
        try:
            with base_limits(self.io, max_vx=limits["max_vx"], max_vy=limits["max_vy"], max_wz=limits["max_wz"],
                             accel=0.5, angular_accel=1.5):
                res = self.nav.goto(x, y, yaw, label=label)
        finally:
            self.nav.drive_enabled = False
            self.io.stop_base()
        self.say("nav to %s: %s (error %s m)" % (label, res["status"], res.get("err_xy_m")))
        if not res["ok"]:
            raise MissionFailed("navigation to %s failed: %s" % (label, res["status"]))
        return res

    def s_deliver(self):
        io = self.io
        d = self.deliver
        if d["kind"] == "hold":                       # assistant mode: keep the verified grasp and look forward for the person
            pose = list(self.cfg["exp"]["people_pose"])
            pose[5] = self.grip_target()
            io.move_arm(pose, check=False)
            self.result = "object grasped and held"
            return "DONE"
        io.move_arm(self.carry_pose(True))
        if d["kind"] == "waypoint":
            w = d["point"]
            self.nav_goto(w["x"], w["y"], w["yaw"], d.get("name", "drop"))
        else:                                          # return to where this run started (odometry, bounded distance)
            rel = se2_between(io.pose(), self.start_pose)
            dist = math.hypot(rel[0], rel[1])
            self.say("back to the start pose: %.2f m away" % dist)
            if dist > 6.0:
                raise MissionFailed("start pose is %.1f m away: too far for odometry-only return" % dist)
            if dist > 0.15:
                turn_by(io, math.atan2(rel[1], rel[0]), wz=0.5)
                translate(io, dist, 0.0, speed=0.10, timeout=40.0)
        size = self.cube_cfg["size_m"]
        target = np.array([0.22, 0.0, size / 2 + 0.004])
        self.move_to_tcp(target + np.array([0, 0, 0.08]), seed=self.carry_pose(True))
        self.move_to_tcp(target, seed=io.arm_command(), duration_ms=1200)
        self.open_gripper()
        io.wait(0.6)
        self.move_to_tcp(target + np.array([0, 0, 0.10]), seed=io.arm_command())
        io.move_arm(self.carry_pose(False))
        self.drop_spot = target[:2]
        return "VERIFY"

    def s_verify(self):
        """The delivery counts only if the camera sees the object on the floor where it was put and not in the hand."""
        io = self.io
        io.move_arm([int(v) for v in self.arm_cfg["near_search_pose"]])
        seen = None
        for _ in range(3):
            seen = self.see_cube(hint_odom=self.odom_from_base(self.drop_spot), gate=0.15)
            if seen is not None:
                break
        io.move_arm(self.search_pose())
        if seen is None:
            raise MissionFailed("delivery not confirmed: the object is not visible at the drop spot")
        err = float(np.linalg.norm(seen - self.drop_spot))
        self.say("object seen on the floor at %s (%.0f mm from the intended spot)" % (np.round(seen, 3), err * 1e3))
        self.stages["delivered"] = True
        self.result = "delivered and confirmed by the camera (%.0f mm from the target)" % (err * 1e3)
        return "DONE"


class RetrieveExperiment(Experiment):
    key = "e4_retrieve"
    title = "4. Найти и принести предмет"

    def __init__(self, ctx, nav_getter):
        super().__init__(ctx)
        self.nav_getter = nav_getter
        self.mission = None

    def setup(self):
        text = self.params.get("target", "кубик")
        self.label = resolve_target(text, self.ecfg) or self.params.get("target", "cube")
        self.allow_runtime(float(self.params.get("runtime_s", 900)))
        self.ctx.perception.set_want(people=False, objects=False)
        self.state = "running"
        self.say("цель: «%s» → %s" % (text, self.label))

    def run(self):
        io = self.io
        nav = self.nav_getter(create=False)
        d = dict(kind="start")
        if nav is not None and nav.running:
            name = self.params.get("deliver", "drop")
            if name in nav.waypoints:
                d = dict(kind="waypoint", name=name, point=nav.waypoints[name])
        self.say("доставка: %s" % ("точка «%s» на карте" % d["name"] if d["kind"] == "waypoint" else "стартовая поза по одометрии"))
        self.mission = Retriever(io, self.cfg, self.say, self.label, d, self.ctx.perception,
                                 nav if nav is not None and nav.running else None)
        t0 = time.monotonic()
        ok = self.mission.run()
        m = self.mission
        rec = dict(kind="trial", target=self.label, requested=self.params.get("target"), ok=bool(ok), result=m.result,
                   fail_reason=m.fail_reason, stages=m.stages, seconds=round(time.monotonic() - t0, 1),
                   delivery=d["kind"], object_note=self.params.get("note", ""), video=self._recorder.path)
        self.ctx.store.add(self.key, rec)
        self.result = m.result
        self.state = "done" if ok else "failed"
        self.metrics = dict(stages=m.stages, state=m.state)

    def live(self):
        m = self.mission
        return dict(stage=None if m is None else m.state, stages=None if m is None else m.stages)

    def overlay(self, img):
        m = self.mission
        put_text(img, "E4 %s stage=%s %s" % (getattr(self, "label", "?"), None if m is None else m.state,
                                             None if m is None else m.stages), (8, 20), (0, 255, 255))
        return img
