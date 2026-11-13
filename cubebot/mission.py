"""Mission state machine: find the cube, put it in the box, take it out again.

The arm reports no joint angles, so every arm pose is a command estimate. The camera
is on the wrist (fixed relative to the gripper), which makes the cube position
*relative to the gripper* directly measurable. Grasping therefore closes the loop
in the camera frame (visual servoing) instead of trusting absolute kinematics.
"""
import math
import os
import threading

import numpy as np

from .geometry import se2_apply, se2_inverse, wrap
from .robot_io import MissionAbort
from .vision import (cube_in_hand, deproject, detect_box, detect_box_by_color, detect_cubes, fit_floor, learn_box_color,
                     gravity_aligned_pose, to_base)

HOVER_STAGES = (0.16, 0.10, 0.05, 0.0)   # axial distance TCP -> cube centre during the approach


class MissionFailed(Exception):
    pass


class Look:
    def __init__(self, frame, t_base_cam, floor, servo):
        self.frame, self.t_base_cam, self.floor, self.servo = frame, t_base_cam, floor, servo


class Mission:
    def __init__(self, io, cfg, log=print):
        self.io, self.cfg, self.arm, self.log = io, cfg, io.arm, log
        self.cube_cfg, self.cam_cfg, self.box_cfg = cfg["cube"], cfg["camera"], cfg["box"]
        self.arm_cfg, self.m_cfg, self.base_cfg = cfg["arm"], cfg["mission"], cfg["base"]
        self.cube_odom = None          # xy in odom, with hit count
        self.cube_hits = 0
        self.cube_released = True          # False while the cube is held (set after a verified grasp)
        self.box = None                # dict(center, yaw, length, width, rim) in odom
        self._rng = np.random.default_rng(0)
        self._windows = {}
        self.state = "PREFLIGHT"
        self.result = None
        self.hand_roi = None
        self._sizes = []

    # ---------------------------------------------------------------- utilities
    def save_debug(self, frame, obs, limit=6, prefix="candidate"):
        """Keep a few annotated frames in /tmp for after-the-fact review (obs may be None)."""
        counters = self.__dict__.setdefault("_debug_saved", {})
        if counters.get(prefix, 0) >= limit:
            return
        try:
            import cv2
            img = cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2BGR)
            if obs is not None:
                cv2.circle(img, (int(obs.u), int(obs.v)), 14, (0, 0, 255), 2)
            counters[prefix] = counters.get(prefix, 0) + 1
            cv2.imwrite("/tmp/cubebot_%s_%02d.png" % (prefix, counters[prefix]), img)
        except Exception:
            pass

    def resume_with_cube(self):
        """Continue a mission whose cube is already clamped in the gripper (skips search and grasp)."""
        self.cube_released = False
        self.grasp_spot = np.array([0.22, 0.0])
        self.state = "SEARCH_BOX"

    def say(self, text):
        self.log("[%s] %s" % (self.state, text))

    def search_pose(self):
        return list(self.arm_cfg["search_pose"])

    def carry_pose(self, holding):
        pose = list(self.arm_cfg["search_pose"])
        pose[5] = self.grip_target() if holding else self.arm_cfg["gripper_open"]
        return pose

    def window(self, z):
        """(x_min, x_max) of floor-referenced TCP targets reachable straight ahead, cached."""
        key = round(z, 4)
        if key not in self._windows:
            self._windows[key] = self.arm.reach_window(z, self.search_pose(), cache_files=self.cache_files())
        return self._windows[key]

    @staticmethod
    def cache_files():
        here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        user = os.path.expanduser("~/.cache/cubebot")
        try:
            os.makedirs(user, exist_ok=True)
        except OSError:
            pass
        return (os.path.join(here, "config", "reach_cache.json"), os.path.join(user, "reach_cache.json"))

    def look(self, settle=0.35):
        io = self.io
        io.wait_arm()
        io.wait(settle)
        frame = None
        for _ in range(4):
            frame = io.frame_after(io.now() - 0.02, timeout=2.5)
            if frame is not None:
                break
            self.say("waiting for a camera frame")
        if frame is None:
            raise MissionFailed("no camera frame")
        servo = io.arm_command()
        t_fk = self.arm.camera_pose(servo)
        down_pred = t_fk[:3, :3].T @ np.array([0., 0., -1.])
        floor = fit_floor(frame.depth, frame.K, down_pred, self.cam_cfg, self._rng)
        t_cam = t_fk
        if floor is not None:
            tilt = math.degrees(math.acos(max(-1., min(1., float(floor.down @ down_pred)))))
            if tilt < 14.0 and abs(floor.height - t_fk[2, 3]) < 0.08:
                t_cam = gravity_aligned_pose(t_fk, floor)
            else:
                self.say("floor plane disagrees with FK (tilt %.1f deg, dz %.3f m); using FK" %
                         (tilt, floor.height - t_fk[2, 3]))
        return Look(frame, t_cam, floor, servo)

    def cube_base(self, look, obs):
        """Base-frame cube centre: ray/floor intersection when the geometry is good, else depth."""
        t = look.t_base_cam
        K = look.frame.K
        origin = t[:3, 3]
        ray = t[:3, :3] @ np.array([(obs.u - K[0, 2]) / K[0, 0], (obs.v - K[1, 2]) / K[1, 1], 1.0])
        zc = self.cube_cfg["size_m"] / 2
        if ray[2] < -0.12 and look.floor is not None:
            s = (zc - origin[2]) / ray[2]
            hit = origin + s * ray
            depth_pt = to_base(t, obs.cam_xyz)
            if s > 0 and (not obs.depth_valid or np.linalg.norm(hit[:2] - depth_pt[:2]) < 0.06):
                return hit
        return to_base(t, obs.cam_xyz)

    def odom_from_base(self, xy):
        return se2_apply(self.io.pose(), np.asarray(xy)[None, :])[0]

    def base_from_odom(self, xy):
        return se2_apply(se2_inverse(self.io.pose()), np.asarray(xy)[None, :])[0]

    def learn_size(self, look, observations):
        """Replace the size guess by the cube height measured from depth.

        The top face of a cube lies at height = edge length above the floor, so the 90th
        percentile of the blob's base-frame z is a direct, perspective-free measurement.
        Needs a camera pose anchored to the floor plane.
        """
        c = self.cube_cfg
        if look.floor is None:
            return
        for obs in observations:
            if obs.pts_cam is None or obs.range_m > 1.3:
                continue
            z = (look.t_base_cam[:3, :3] @ obs.pts_cam.T).T[:, 2] + look.t_base_cam[2, 3]
            height = float(np.percentile(z, 90))
            if c["size_min_m"] < height < c["size_max_m"]:
                self._sizes.append(height)
        if len(self._sizes) >= 5:
            measured = float(np.median(self._sizes[-15:]))
            if abs(measured - c["size_m"]) > 0.002:
                self.say("cube height measured from depth: %.1f mm (was %.1f mm)" % (measured * 1e3, c["size_m"] * 1e3))
                c["size_m"] = measured

    def remember_cube(self, base_xy):
        odom = self.odom_from_base(base_xy)
        if self.cube_odom is not None and np.linalg.norm(odom - self.cube_odom) < 0.08:
            k = min(self.cube_hits, 4)
            self.cube_odom = (self.cube_odom * k + odom) / (k + 1)
            self.cube_hits += 1
        else:
            self.cube_odom, self.cube_hits = odom, 1

    def remember_box(self, box):
        # Close to the box its near wall leaves the camera view and the visible rim shrinks, which
        # would drag the estimate towards the robot. From then on odometry carries the memory.
        if self.box is not None and float(np.linalg.norm(self.box_center_base())) < self.box_cfg["freeze_within_m"]:
            return
        pose = self.io.pose()
        center = self.odom_from_base(box.center_xy)
        entry = dict(center=center, yaw=box.yaw + pose[2], length=box.length, width=box.width, rim=box.rim_height,
                     near=self.odom_from_base(box.near_xy))
        if self.box is None or np.linalg.norm(self.box["center"] - center) > 0.1:
            self.box = entry
        else:
            for key in ("center", "yaw", "length", "width", "rim"):
                self.box[key] = 0.5 * np.asarray(self.box[key]) + 0.5 * np.asarray(entry[key])
            self.box["near"] = entry["near"]

    def scan_scene(self, want_cube=True, want_box=True):
        """One look from the current spot: update cube and box memory."""
        look = self.look(settle=self.m_cfg["search_settle_s"])
        seen_cube = False
        if want_cube:
            found = detect_cubes(look.frame, self.cube_cfg, self.cam_cfg)
            self.learn_size(look, found)
            for obs in found:
                self.say("candidate: pixel (%.0f,%.0f) width %.0f px range %.2f m depth=%s base %s" % (
                    obs.u, obs.v, obs.width_px, obs.range_m, obs.depth_valid, np.round(self.cube_base(look, obs), 3)))
                self.save_debug(look.frame, obs)
                self.remember_cube(self.cube_base(look, obs)[:2])
                seen_cube = True
                break
        if want_box:
            box = detect_box(look.frame, look.t_base_cam, 0.0, self.box_cfg, self.cam_cfg)
            if box is not None:
                self.box_color = learn_box_color(look.frame, look.t_base_cam, box) or getattr(self, "box_color", None)
                self.remember_box(box)
        return look, seen_cube

    # ---------------------------------------------------------------- base motion
    def turn_by(self, angle, tolerance=0.05, timeout=14.0):
        io = self.io
        target = wrap(io.pose()[2] + angle)

        def cmd():
            err = wrap(target - io.pose()[2])
            wz = max(-self.base_cfg["max_wz"], min(self.base_cfg["max_wz"], 1.6 * err))
            return 0.0, 0.0, math.copysign(max(abs(wz), 0.12), err)       # below ~0.1 rad/s the wheels stall

        io.drive_until(cmd, lambda: abs(wrap(target - io.pose()[2])) < tolerance, timeout, "turn")
        io.wait(0.2)

    def drive_distance(self, direction, distance, speed=0.08, timeout=15.0):
        """Translate along base-frame `direction` (unit xy) for `distance`, stopping early if blocked."""
        io = self.io
        start = np.array(io.pose()[:2])
        direction = np.asarray(direction, float)
        c, s = math.cos(io.pose()[2]), math.sin(io.pose()[2])
        world_dir = np.array([c * direction[0] - s * direction[1], s * direction[0] + c * direction[1]])

        def travelled():
            return float(np.dot(np.array(io.pose()[:2]) - start, world_dir))

        def cmd():
            yaw = io.pose()[2]
            c2, s2 = math.cos(yaw), math.sin(yaw)
            vx = (c2 * world_dir[0] + s2 * world_dir[1]) * speed
            vy = (-s2 * world_dir[0] + c2 * world_dir[1]) * speed
            return vx, vy, 0.0

        ok = io.drive_until(cmd, lambda: travelled() >= distance - 0.01, timeout, "drive")
        io.wait(0.2)
        return ok

    def goto_base_target(self, get_target, standoff, tolerance, timeout, face=True, label="", max_travel=None, lost_after=None):
        """Move so the object (base-frame xy from `get_target()`) sits at `standoff` (x, y)."""
        io = self.io
        goal = np.asarray(standoff, float)
        base = self.base_cfg

        start = io.pose()
        state = {"over": False}

        def error():
            t = get_target()
            return None if t is None else np.asarray(t) - goal

        def travelled():
            p = io.pose()
            return math.hypot(p[0] - start[0], p[1] - start[1])

        def cmd():
            e = error()
            if e is None:
                return 0.0, 0.0, 0.0
            if lost_after is not None and io.now() - getattr(self, "cube_seen", io.now()) > lost_after and e[0] > 0.12:
                if not state.get("lost"):
                    self.say("%s: cube out of sight: waiting for it to reappear" % label)
                state["lost"] = True
                t = np.asarray(get_target())
                return 0.0, 0.0, math.copysign(0.15, t[1] if abs(t[1]) > 0.02 else 1.0)    # look towards where it was
            state["lost"] = False
            if max_travel is not None and travelled() > max_travel:
                if not state["over"]:
                    self.say("%s: travelled %.2f m (limit %.2f m): stopping instead of driving on" % (label, travelled(), max_travel))
                state["over"] = True
                return 0.0, 0.0, 0.0
            t = np.asarray(get_target())
            if io.now() - state.get("log", -9) > 1.0:
                state["log"] = io.now()
                p = io.pose()
                self.say("%s: target %s err %s pose (%.2f, %.2f, %.0f deg) guard %s" % (
                    label, np.round(t, 3), np.round(e, 3), p[0], p[1], math.degrees(p[2]), io.guard.last_block))
            bearing = math.atan2(t[1], max(t[0], 0.05)) if face else 0.0
            dist = float(np.linalg.norm(e))
            gain = 0.9 if dist < 0.3 else 0.6
            vx = max(-base["max_vx"], min(base["max_vx"], gain * e[0]))
            vy = max(-base["max_vy"], min(base["max_vy"], gain * e[1]))
            if dist > 0.4:
                vy *= 0.3
            wz = max(-base["max_wz"], min(base["max_wz"], 1.4 * bearing)) if abs(bearing) > 0.05 else 0.0
            # Large heading error first: avoid translating sideways with the object out of view.
            if abs(bearing) > 0.5:
                vx, vy = 0.0, 0.0
            # The wheels stall at very low speed: keep a minimum while the error is still outside tolerance.
            if abs(e[0]) > tolerance * 0.5:
                vx = math.copysign(max(abs(vx), 0.03), e[0])
            if abs(e[1]) > tolerance * 0.5 and (vx != 0.0 or abs(bearing) <= 0.5):
                vy = math.copysign(max(abs(vy), 0.03), e[1])
            return vx, vy, wz

        def done():
            if state["over"]:
                return True
            e = error()
            return e is not None and np.linalg.norm(e) < tolerance

        return io.drive_until(cmd, done, timeout, label)

    # ---------------------------------------------------------------- states
    def run(self):
        handlers = {
            "PREFLIGHT": self.s_preflight, "SEARCH_CUBE": self.s_search_cube,
            "APPROACH_CUBE": self.s_approach_cube, "GRASP": self.s_grasp,
            "SEARCH_BOX": self.s_search_box, "APPROACH_BOX": self.s_approach_box,
            "PLACE": self.s_place, "EXTRACT": self.s_extract, "FINISH": self.s_finish,
        }
        try:
            while self.state not in ("DONE", "FAILED"):
                self.say("enter")
                self.state = handlers[self.state]()
        except MissionAbort as exc:
            self.result = "aborted: %s" % exc
            self.state = "FAILED"
        except MissionFailed as exc:
            self.result = "failed: %s" % exc
            self.state = "FAILED"
        finally:
            self.io.stop_base()
            try:
                self.io.abort_reason = None
                park = list(self.arm_cfg["safe_pose"])
                if self.state == "FAILED" and getattr(self, "grasp_spot", None) is not None and not self.cube_released:
                    park[5] = int(round(self.io.arm_command()[5]))     # holding the cube: park without dropping it
                self.io.move_arm(park, check=False, wait=False)
            except Exception as exc:  # the arm park must never mask the real result
                self.log("arm park failed: %s" % exc)
        if self.state == "DONE":
            self.result = self.result or "done"
        self.log("mission result: %s" % self.result)
        return self.state == "DONE"

    def s_preflight(self):
        io = self.io
        io.sync_arm_estimate()
        v = io.battery()
        if v is not None and v < self.m_cfg["min_battery_v"] + 0.3:
            raise MissionFailed("battery too low to start (%.2f V)" % v)
        io.stop_base()
        io.move_arm(self.search_pose())
        io.wait(1.5)
        look = self.look()
        if look.floor is None:
            self.say("floor plane not visible: trusting the arm model")
            if self.window(self.cube_cfg["size_m"] / 2) is None:
                raise MissionFailed("arm cannot reach the floor in the model")
            return "SEARCH_CUBE"
        expected = look.servo and self.arm.camera_pose(look.servo)
        down = expected[:3, :3].T @ np.array([0., 0., -1.])
        tilt = math.degrees(math.acos(max(-1., min(1., float(look.floor.down @ down)))))
        dz = look.floor.height - expected[2, 3]
        self.say("self-check: camera tilt error %.1f deg, height error %.3f m, floor inliers %.0f%%" %
                 (tilt, dz, 100 * look.floor.inlier_ratio))
        if (tilt > 12.0 or abs(dz) > 0.06) and look.floor.inlier_ratio < 0.3:
            self.say("floor view is poor (%.0f%% inliers): trusting the arm model" % (100 * look.floor.inlier_ratio))
        elif tilt > 12.0 or abs(dz) > 0.06:
            raise MissionFailed("arm model disagrees with the camera (tilt %.1f deg, dz %.3f m)" % (tilt, dz))
        if self.window(self.cube_cfg["size_m"] / 2) is None:
            raise MissionFailed("arm cannot reach the floor in the model")
        return "SEARCH_CUBE"

    def sweep_for_cube(self, pose):
        """One 360 degree turn in steps with the arm in `pose`; True once the cube is confirmed."""
        io, m = self.io, self.m_cfg
        if list(np.round(io.arm_command()).astype(int)) != list(pose):
            io.move_arm(pose)
        for _ in range(int(round(360 / m["search_turn_deg"]))):
            _, seen = self.scan_scene()
            if seen and self.cube_hits >= 2:
                return True
            if seen:                         # confirm from a second frame
                _, seen2 = self.scan_scene(want_box=False)
                if seen2:
                    return True
            self.turn_by(math.radians(m["search_turn_deg"]))
        return False

    def s_search_cube(self):
        m = self.m_cfg
        for round_index in range(m["max_search_rounds"]):
            # Far view first (floor from ~0.4 m to the horizon), then a steeper view for nearby cubes.
            if self.sweep_for_cube(self.search_pose()) or self.sweep_for_cube(list(self.arm_cfg["near_search_pose"])):
                self.io.move_arm(self.search_pose())
                self.say("cube at odom %s after %d hits" % (np.round(self.cube_odom, 2), self.cube_hits))
                return "APPROACH_CUBE"
            self.io.move_arm(self.search_pose())
            self.say("round %d: nothing found" % (round_index + 1))
        raise MissionFailed("cube not found after a full turn with both camera views")

    def cube_target(self):
        return None if self.cube_odom is None else self.base_from_odom(self.cube_odom)

    def refresh_cube_from_camera(self):
        io = self.io
        frame = io._latest_frame()
        if frame is None:
            return
        servo = io.arm_command()
        t_fk = self.arm.camera_pose(servo)
        down_pred = t_fk[:3, :3].T @ np.array([0., 0., -1.])
        floor = fit_floor(frame.depth, frame.K, down_pred, self.cam_cfg, self._rng)
        t = gravity_aligned_pose(t_fk, floor) if floor is not None else t_fk
        look = Look(frame, t, floor, servo)
        target = self.cube_target()
        found = detect_cubes(frame, self.cube_cfg, self.cam_cfg)
        self.learn_size(look, found)
        if io.now() - getattr(self, "_dbg_t", -9) > 1.0:
            self._dbg_t = io.now()
            self.say("refresh: %d detections %s target %s" % (len(found), [(round(o.u), round(o.v), round(o.range_m, 2)) for o in found][:3],
                                                              None if target is None else np.round(target, 2)))
        for obs in found:
            base = self.cube_base(look, obs)
            if target is None or np.linalg.norm(base[:2] - target) < 0.5:
                odom = self.odom_from_base(base[:2])
                near = target is not None and np.linalg.norm(base[:2] - target) < 0.15
                # Odometry drifts while driving: when the camera disagrees, the camera wins.
                self.cube_odom = 0.7 * self.cube_odom + 0.3 * odom if (self.cube_odom is not None and near) else odom
                self.cube_seen = self.io.now()
                return

    def stand_off(self, z):
        win = self.window(z)
        if win is None:
            raise MissionFailed("cube height is not reachable by the arm")
        return 0.5 * (win[0] + win[1])

    def background(self, fn, period):
        """Return poll(): runs `fn` about every `period` s without stalling the 20 Hz drive loop (inline in the simulator)."""
        io, state = self.io, {"t": -1e9, "busy": False}

        def work():
            try:
                fn()
            finally:
                state["busy"] = False

        def poll():
            if state["busy"] or io.now() - state["t"] < period:
                return
            state["t"], state["busy"] = io.now(), True
            if io.REALTIME:
                threading.Thread(target=work, daemon=True).start()
            else:
                work()
        return poll

    def see_cube(self, hint_odom=None, gate=0.3):
        """One fresh camera look: base-frame xy of the cube (nearest to the expected spot), or None."""
        look = self.look(settle=0.2)
        found = detect_cubes(look.frame, self.cube_cfg, self.cam_cfg)
        self.learn_size(look, found)
        best, best_d = None, 1e9
        if hint_odom is not None:
            gate += 0.3 * float(np.linalg.norm(self.base_from_odom(hint_odom)))      # far ranges are less accurate
        for obs in found:
            xy = self.cube_base(look, obs)[:2]
            d = 0.0 if hint_odom is None else float(np.linalg.norm(xy - self.base_from_odom(hint_odom)))
            if d < best_d:
                best, best_d = xy, d
        if best is None or (hint_odom is not None and best_d > gate):
            return None
        self.cube_odom = self.odom_from_base(best)
        return best

    def s_approach_cube(self):
        """Stop-and-go visual servoing: look, make one short move, look again. The camera is the only reference,
        odometry is trusted just for the few centimetres between two looks."""
        io = self.io
        sx = self.stand_off(self.cube_cfg["size_m"] / 2)
        goal = np.array([sx, 0.0])
        near = [int(v) for v in self.arm_cfg["near_search_pose"]]
        io.guard.mode = "dock"
        hint, misses, last_err = self.cube_odom, 0, None
        if hint is not None and self.base_from_odom(hint)[0] < 0.42:
            io.move_arm(near)                      # a close cube is only visible in the steep view
        try:
            for step in range(80):
                t = self.see_cube(hint)
                if t is None:
                    if last_err is not None and last_err < 0.09 and misses == 0:
                        self.say("approach: the cube slipped below the camera view at the stand-off distance: arrived")
                        return "GRASP"
                    misses += 1
                    if misses > 6:
                        raise MissionFailed("lost the cube during the approach")
                    if misses == 2:                          # the other view may see it: toggle far/near
                        other = self.search_pose() if [int(round(v)) for v in io.arm_command()] == near else near
                        self.say("approach: trying the other camera view")
                        io.move_arm(other)
                    h = None if hint is None else self.base_from_odom(hint)
                    b = 0.0 if h is None else math.atan2(h[1], max(h[0], 0.05))
                    self.say("approach: cube not visible (%d); turning towards where it was" % misses)
                    self.turn_by(max(-0.25, min(0.25, b)) if abs(b) > 0.05 else 0.15 * (-1) ** misses)
                    continue
                misses, hint = 0, self.odom_from_base(t)
                if t[0] < 0.42 and [int(round(v)) for v in io.arm_command()] != near:
                    self.say("approach: cube is close, switching to the near view")
                    io.move_arm(near)
                    continue
                e = t - goal
                last_err = float(np.linalg.norm(e))
                self.say("approach %d: cube at %s, error %s" % (step, np.round(t, 3), np.round(e, 3)))
                if last_err < 0.035:
                    return "GRASP"
                bearing = math.atan2(t[1], max(t[0], 0.05))
                if abs(bearing) > 0.35:                     # small bearings are cheaper to fix by strafing (mecanum)
                    self.turn_by(max(-0.3, min(0.3, bearing)))
                else:
                    dist = float(np.linalg.norm(e))
                    self.drive_distance(e / dist, min(0.07, dist), speed=0.06, timeout=3.5)
            raise MissionFailed("could not reach the cube stand-off position")
        finally:
            io.guard.mode = "normal"
            io.stop_base()

    # ---------------------------------------------------------------- grasping
    def arm_path_ok(self, servo):
        if not self.arm.within_limits(servo) or not self.arm.collision_free(servo, self.arm_cfg["floor_margin_m"]):
            return False
        if self.box is None:
            return True
        return not self.hits_box_wall(servo)

    def hits_box_wall(self, servo):
        """True when an arm link point (not the fingertips) is inside a box wall slab."""
        box = self.box
        center = self.base_from_odom(box["center"])
        yaw = box["yaw"] - self.io.pose()[2]
        c, s = math.cos(-yaw), math.sin(-yaw)
        pts = self.arm.link_points(servo)[:-6]          # exclude the last (TCP) segment
        local = (pts[:, :2] - center) @ np.array([[c, s], [-s, c]]).T
        z = pts[:, 2]
        half_l, half_w = box["length"] / 2, box["width"] / 2
        wall = 0.02
        near_wall = (((np.abs(local[:, 0]) > half_l - wall) & (np.abs(local[:, 0]) < half_l + 0.005)
                      & (np.abs(local[:, 1]) < half_w)) |
                     ((np.abs(local[:, 1]) > half_w - wall) & (np.abs(local[:, 1]) < half_w + 0.005)
                      & (np.abs(local[:, 0]) < half_l)))
        return bool(np.any(near_wall & (z < box["rim"] + 0.005)))

    def move_to_tcp(self, target, seed=None, pitches=None, duration_ms=None):
        servo = seed or self.io.arm_command()
        sol, pitch = self.arm.grasp_ik(target, servo, pitches=pitches)
        if sol is None:
            raise MissionFailed("IK has no solution for %s" % np.round(target, 3))
        full = list(np.round(sol).astype(int)) + [int(round(self.io.arm_command()[5]))]
        if not self.arm_path_ok(full):
            raise MissionFailed("arm pose %s would collide" % full)
        self.io.move_arm(full, duration_ms=duration_ms, check=False)
        return full

    def pick_obs(self, look, expected_cam=None):
        obs = detect_cubes(look.frame, self.cube_cfg, self.cam_cfg)
        if not obs:
            return None
        if expected_cam is None:
            return obs[0]
        return min(obs, key=lambda o: np.linalg.norm(o.cam_xyz - expected_cam))

    def servo_cube(self, hint_base, stages=HOVER_STAGES):
        """Visual servoing in the camera frame; returns True with the TCP at the cube centre."""
        io, arm = self.io, self.arm
        size = self.cube_cfg["size_m"]
        cube = np.array([hint_base[0], hint_base[1], max(size / 2, 0.02)])
        bearing = math.atan2(cube[1], cube[0] - 0.034)
        for pitch in (70, 60, 80, 50, 90):
            axis = np.array([math.cos(math.radians(pitch)) * math.cos(bearing),
                             math.cos(math.radians(pitch)) * math.sin(bearing), -math.sin(math.radians(pitch))])
            sol, _ = self.arm.ik(cube - stages[0] * axis, pitch, self.search_pose())
            if sol is not None:
                break
        else:
            raise MissionFailed("no hover pose for the cube at %s" % np.round(cube, 3))
        self.move_to_tcp(cube - stages[0] * axis, seed=self.search_pose(), pitches=(pitch,))
        tcp_cam = arm.tcp_in_camera(io.arm_command())[:3]
        expected = tcp_cam + np.array([0., 0., stages[0]])
        if not self.bring_cube_into_view(cube, expected):
            self.say("cube not visible from the hover pose even after re-aiming the camera")
            return False
        pred = None                                   # predicted cube position in the camera frame
        for h in stages:
            desired = tcp_cam + np.array([0., 0., h])
            for it in range(10):
                look = self.look(settle=0.45)
                obs = self.pick_obs(look, expected)
                K = look.frame.K
                self.save_debug(look.frame, obs, limit=40, prefix="servo")
                if obs is None and pred is None:
                    return False
                if obs is None:
                    meas = pred.copy()
                else:
                    if obs.depth_valid:
                        z = obs.range_m
                    else:
                        z = pred[2] if pred is not None else obs.range_m
                    x = (obs.u - K[0, 2]) * z / K[0, 0]
                    y = (obs.v - K[1, 2]) * z / K[1, 1]
                    if obs.clipped and pred is not None:
                        y = pred[1]                    # the blob is cut off: vertical position is not measured
                    meas = np.array([x, y, z])
                # Calibration bias (negative x = grasp further ahead): the fingers closed slightly behind the cube.
                bias_cam = look.t_base_cam[:3, :3].T @ np.asarray(self.arm_cfg.get("grasp_bias_base_m", (0., 0., 0.)))
                desired_eff = desired + bias_cam
                err = desired_eff - meas
                lateral = float(np.linalg.norm(err[:2]))
                self.say("stage h=%.2f it=%d lateral=%.1f mm axial=%.1f mm%s" % (
                    h, it, lateral * 1e3, err[2] * 1e3,
                    "" if obs is not None else " (blind)" if True else ""))
                tol_lat = 0.007 if h > 0 else 0.008
                if lateral < tol_lat and abs(err[2]) < 0.012:
                    pred = meas
                    expected = desired
                    break
                if obs is None and it >= 1:
                    pred = meas
                    break
                servo = np.array(io.arm_command(), float)
                t_old = arm.camera_pose(servo)
                move = look.t_base_cam[:3, :3] @ (meas - desired_eff)    # TCP shift that fixes the error
                move = move * min(1.0, 0.06 / max(1e-6, float(np.linalg.norm(move))))
                tcp_now = arm.tcp_position(servo)
                target = tcp_now + move
                pitch_now = math.degrees(arm.approach_pitch_down(servo))
                sol, ik_err = arm.ik(target, pitch_now, servo, stay_weight=0.0004)
                if sol is None:                      # the current tilt may not be reachable any more
                    for pitch in (pitch_now - 10, pitch_now + 10, pitch_now - 20, pitch_now + 20):
                        sol, ik_err = arm.ik(target, pitch, servo, stay_weight=0.0004)
                        if sol is not None:
                            break
                if sol is None:
                    self.say("no IK for the corrective move: tcp %s + move %s -> %s (best error %.3f m, pitch %.0f)" % (
                        np.round(tcp_now, 3), np.round(move, 3), np.round(target, 3), ik_err, pitch_now))
                    return False
                new = np.round(np.concatenate([sol[:4], servo[4:]])).astype(int)
                if np.array_equal(new, np.round(servo).astype(int)):
                    pred = meas
                    break
                if not self.arm_path_ok(new):
                    self.say("servo step rejected (limits/collision)")
                    return False
                io.move_arm(list(new), duration_ms=500, check=False)
                point_base = (t_old @ np.append(meas, 1.0))[:3]
                pred = (np.linalg.inv(arm.camera_pose(new)) @ np.append(point_base, 1.0))[:3]
                expected = desired
            else:
                self.say("stage h=%.2f did not converge" % h)
                if h > 0.05:
                    return False
        return True

    def bring_cube_into_view(self, cube_base, expected_cam, tries=5):
        """Servo errors of several degrees can leave the cube outside the image after the hover move.

        The camera pose is anchored to the floor plane, so the cube's expected camera-frame position is
        known; J1 (yaw) and J4 (wrist pitch) are nudged until the cube appears.
        """
        io, arm = self.io, self.arm
        for k in range(tries):
            look = self.look(settle=0.45)
            if self.pick_obs(look, expected_cam) is not None:
                return True
            servo = np.array(io.arm_command(), float)
            p_cam = (np.linalg.inv(look.t_base_cam) @ np.append(cube_base, 1.0))[:3]
            self.say("cube not in view; expected at camera %s (floor %s)" % (
                np.round(p_cam, 3), "ok" if look.floor is not None else "missing"))
            if p_cam[2] < 0.05:
                return False
            jac = arm.camera_jacobian_deg(servo, cube_base)[:2][:, [0, 3]]
            try:
                dq = np.linalg.solve(jac + 1e-6 * np.eye(2), -p_cam[:2])
            except np.linalg.LinAlgError:
                return False
            dq = np.clip(dq, -12.0, 12.0)
            new = servo.copy()
            new[0] += dq[0]
            new[3] += dq[1]
            new = np.round(new).astype(int)
            if np.array_equal(new, np.round(servo).astype(int)) or not self.arm_path_ok(new):
                return False
            io.move_arm(list(new), duration_ms=600, check=False)
        return self.pick_obs(self.look(settle=0.45), expected_cam) is not None

    def grip_target(self):
        """Gripper command for holding the cube: estimated contact angle from the measured size, plus an overdrive.

        The servo is a position servo: its torque grows with the gap between the command and the real angle, so a
        command well past the contact angle is what clamps the cube. The contact angle follows from the cube width
        (jaw gap linear in the angle: gripper_width_open_m at gripper_open, 0 at gripper_close); the overdrive and
        the +5 deg after every failed attempt are capped by gripper_hold_max (servo limit is 170 deg).
        """
        a = self.arm_cfg
        span = a["gripper_close"] - a["gripper_open"]
        width = min(max(self.cube_cfg["size_m"], 0.01), a["gripper_width_open_m"])
        contact = a["gripper_close"] - width / a["gripper_width_open_m"] * span
        return int(round(min(a["gripper_hold_max"], contact + a["gripper_overdrive_deg"] + getattr(self, "grip_extra", 0))))

    def close_gripper(self):
        io, a = self.io, self.arm_cfg
        pose = io.arm_command()
        value = int(pose[5])
        goal = self.grip_target()
        self.say("closing the gripper to %d deg for a %.1f mm cube" % (goal, self.cube_cfg["size_m"] * 1e3))
        while value < goal:
            value = min(goal, value + a["gripper_close_step"])
            pose[5] = value
            io.move_arm(pose, duration_ms=180, check=False)
        io.wait(0.5)

    def open_gripper(self):
        self.cube_released = True
        pose = self.io.arm_command()
        pose[5] = self.arm_cfg["gripper_open"]
        self.io.move_arm(pose, duration_ms=500, check=False)

    def update_hand_roi(self, K, shape):
        tcp_cam = self.arm.tcp_in_camera(self.io.arm_command())[:3]
        half = self.cube_cfg["size_m"] / 2
        us, vs = [], []
        for dx in (-half, half):
            for dy in (-half, half):
                p = tcp_cam + np.array([dx, dy, 0.0])
                us.append(K[0, 2] + K[0, 0] * p[0] / p[2])
                vs.append(K[1, 2] + K[1, 1] * p[1] / p[2])
        h, w = shape[:2]
        r0, r1 = int(max(0, min(vs))), int(min(h, max(vs)))
        c0, c1 = int(max(0, min(us))), int(min(w, max(us)))
        self.hand_roi = (r0, r1, c0, c1) if r1 - r0 >= 4 and c1 - c0 >= 4 else None

    def gripper_verdict(self):
        """True: the fingers stopped on something; False: they closed all the way; None: no feedback."""
        self.io.wait(0.6)
        if self.io.arm_command()[5] <= self.arm_cfg["gripper_open"] + 10:
            return False                    # commanded open: the fingers hold nothing
        m = self.io.measured_arm()
        if m is None:
            return None
        gap = self.grip_target() - m[5]
        self.say("gripper feedback: %.0f deg (commanded %d), gap %.0f" % (m[5], self.grip_target(), gap))
        if not self.arm_cfg.get("gripper_feedback_valid", False):
            return None          # the gripper servo reports its goal, not the real angle: decide from the camera instead
        if gap > 6.0:
            return True
        if gap < 3.0:
            return False
        return None

    def holding_cube(self):
        verdict = self.gripper_verdict()
        if verdict is not None:
            return verdict
        spot = getattr(self, "grasp_spot", None)
        if spot is not None:
            # The camera cannot see the fingertips: check the floor instead. Lifted, the arm looks at the spot the
            # cube came from; a cube still lying there means the grasp failed, an empty spot means it is in the hand.
            pose = [int(v) for v in self.arm_cfg["near_search_pose"]]
            pose[5] = int(round(self.io.arm_command()[5]))
            self.io.move_arm(pose)
            look = self.look(settle=0.4)
            for obs in detect_cubes(look.frame, self.cube_cfg, self.cam_cfg):
                xy = self.cube_base(look, obs)[:2]
                if float(np.linalg.norm(xy - np.asarray(spot))) < 0.06:
                    self.say("a cube is still lying at the grasp spot %s" % np.round(xy, 3))
                    return False
            self.say("the grasp spot is empty: the cube is in the hand")
            return True
        look = self.look(settle=0.3)
        if self.hand_roi is None:
            self.update_hand_roi(look.frame.K, look.frame.rgb.shape)
        if self.hand_roi is None:
            return None            # gripper area is outside the image: cannot tell
        return cube_in_hand(look.frame.rgb, look.frame.depth, self.hand_roi, self.cube_cfg)

    def reposition_for_grasp(self):
        """The approach already used the camera; the odometry-carried cube position is only a hint for the arm,
        whose own visual servoing corrects a few centimetres. Never chase that hint with the chassis."""
        win = self.window(self.cube_cfg["size_m"] / 2)
        t = self.cube_target()
        if t is not None and not (win[0] - 0.04 <= t[0] <= win[1] + 0.04 and abs(t[1]) < 0.09):
            self.say("cube hint %s is outside the arm window %s: relying on the arm's visual servoing" % (
                np.round(t, 3), np.round(win, 3)))

    def s_grasp(self):
        io, a = self.io, self.arm_cfg
        for attempt in range(1, self.m_cfg["grasp_retries"] + 1):
            self.say("attempt %d" % attempt)
            self.reposition_for_grasp()
            io.move_arm(self.search_pose())
            look = self.look()
            target = None
            obs = self.pick_obs(look)
            if obs is not None:
                base = self.cube_base(look, obs)
                self.remember_cube(base[:2])
                target = base
            elif self.cube_odom is not None:
                b = self.cube_target()
                target = np.array([b[0], b[1], self.cube_cfg["size_m"] / 2])
            if target is None:
                raise MissionFailed("cube lost")
            target = np.array([target[0], target[1], self.cube_cfg["size_m"] / 2])
            self.open_gripper()
            try:
                converged = self.servo_cube(target)
            except MissionFailed as exc:
                self.say("servoing failed: %s" % exc)
                converged = False
            if not converged:
                continue
            self.grasp_spot = np.array(target[:2], float)
            self.close_gripper()
            # Lift straight up, then go to the carry pose and check the hand.
            tcp = self.arm.tcp_position(io.arm_command())
            lift = np.array([tcp[0], tcp[1], tcp[2] + a["lift_height_m"]])
            self.move_to_tcp(lift)
            io.move_arm(self.carry_pose(True))
            holding = self.holding_cube()
            self.say("in-hand check: %s" % holding)
            self.cube_released = holding is False
            if holding is False:
                self.grip_extra = getattr(self, "grip_extra", 0) + 5        # squeeze harder next time
                self.open_gripper()
                io.move_arm(self.search_pose())
                continue
            return "SEARCH_BOX"
        raise MissionFailed("grasp failed after %d attempts" % self.m_cfg["grasp_retries"])

    # ---------------------------------------------------------------- box
    def box_center_base(self):
        return None if self.box is None else self.base_from_odom(self.box["center"])

    def box_near_base(self):
        """Nearest rim point on the line from the robot to the box centre (base frame)."""
        center = self.box_center_base()
        u = center / max(float(np.linalg.norm(center)), 1e-6)
        near = self.base_from_odom(self.box["near"])
        return u * float(near @ u)

    def box_target(self):
        """Drop point: on the line to the box centre, `drop_x_m` ahead of the base origin."""
        center = self.box_center_base()
        if center is None:
            return None
        dist = float(np.linalg.norm(center))
        # Over the middle of the box, as far out as the arm comfortably reaches (never short of drop_x_m).
        return center / max(dist, 1e-3) * min(max(dist, self.box_cfg["drop_x_m"]), self.box_cfg["drop_max_m"])

    def s_search_box(self):
        io = self.io
        io.move_arm(self.carry_pose(True))
        self.box = None                      # whatever was remembered before the grasp is stale: the base has moved
        steps = int(round(360 / self.m_cfg["search_turn_deg"]))
        near = [int(v) for v in self.arm_cfg["near_search_pose"]]
        near[5] = self.carry_pose(True)[5]
        # Far view first; a box standing close to the robot is only visible in the steeper near view.
        for pose in (self.carry_pose(True), near):
            if not self.arm_path_ok(pose):
                continue
            io.move_arm(pose)
            for _ in range(steps):
                self.scan_scene(want_cube=False)
                if self.box is not None:
                    self.box_view_near = pose is near          # a close box is only visible in the steep view
                    return "APPROACH_BOX"
                self.turn_by(math.radians(self.m_cfg["search_turn_deg"]))
        raise MissionFailed("box not found after a full turn with both camera views")

    def refresh_box_from_camera(self):
        frame = self.io._latest_frame()
        if frame is None:
            return
        servo = self.io.arm_command()
        t_fk = self.arm.camera_pose(servo)
        down = t_fk[:3, :3].T @ np.array([0., 0., -1.])
        floor = fit_floor(frame.depth, frame.K, down, self.cam_cfg, self._rng)
        t = gravity_aligned_pose(t_fk, floor) if floor is not None else t_fk
        box = detect_box(frame, t, 0.0, self.box_cfg, self.cam_cfg)
        if box is not None:
            self.remember_box(box)

    def see_box(self, tries=3, expect=None):
        """The box as seen right now (base frame), or None. The detector misses on some frames, so a standing robot
        looks up to `tries` times and keeps the detection closest to the median of the hits."""
        hits = []
        for _ in range(tries):
            look = self.look(settle=0.25)
            box = detect_box(look.frame, look.t_base_cam, 0.0, self.box_cfg, self.cam_cfg)
            if box is not None:
                self.box_color = learn_box_color(look.frame, look.t_base_cam, box) or getattr(self, "box_color", None)
            elif getattr(self, "box_color", None) is not None:
                box = detect_box_by_color(look.frame, look.t_base_cam, self.box_color)
                if box is not None and expect is not None and float(np.linalg.norm(box.near_xy - expect)) > 0.3:
                    box = None                       # a similar-coloured thing elsewhere: not the box we are driving to
                if box is not None:
                    self.say("box found by its colour: rim at %s" % np.round(box.near_xy, 3))
            if box is not None:
                hits.append(box)
                if len(hits) >= 2:
                    break
        if not hits:
            return None
        med = np.median(np.array([b.center_xy for b in hits]), axis=0)
        return min(hits, key=lambda b: float(np.linalg.norm(np.asarray(b.center_xy) - med)))

    def s_approach_box(self):
        """Close the distance to the box with the camera in the loop; no memory is trusted across turns.

        Each step: look; turn once if the box is far off to the side (turning is what makes odometry drift);
        otherwise translate towards the box centre (mecanum: no turn needed). Near the box its wall leaves the view,
        so the last good measurement is carried by odometry for a few short blind steps.
        """
        io = self.io
        io.guard.mode = "dock"
        sx = self.box_cfg["near_edge_m"]
        ok, blind, lost = False, 0, 0
        anchor = None                                   # (centre, nearest rim point, last detection) in odom
        far_pose = self.carry_pose(True)                # cube stays gripped in both views
        near_pose = [int(v) for v in self.arm_cfg["near_search_pose"]]
        near_pose[5] = far_pose[5]
        use_near = bool(getattr(self, "box_view_near", False))
        io.move_arm(near_pose if use_near else far_pose)
        try:
            for step in range(60):
                io.wait(0.2)
                box = self.see_box(expect=None if anchor is None else self.base_from_odom(anchor[1]))
                if box is not None and float(np.linalg.norm(box.center_xy)) > 0.45 or (box is not None and anchor is None):
                    anchor = (self.odom_from_base(box.center_xy), self.odom_from_base(box.near_xy), box)
                    blind = 0
                elif anchor is None:
                    lost += 1
                    if lost > 12:
                        raise MissionFailed("lost the box during the approach")
                    if lost % 2 == 1:                       # maybe the other view sees it
                        use_near = not use_near
                        self.say("approach box: trying the %s camera view" % ("near" if use_near else "far"))
                        io.move_arm(near_pose if use_near else far_pose)
                        continue
                    self.say("approach box: not visible, turning to look for it")
                    if self.box is not None:                 # the search memory says roughly where it is
                        c = self.box_center_base()
                        self.turn_by(max(-0.5, min(0.5, math.atan2(c[1], max(c[0], 0.05)))) if abs(c[1]) > 0.05 else 0.3)
                    else:
                        self.turn_by(0.3)
                    continue
                else:
                    blind += 1
                    if blind > (6 if float(np.linalg.norm(self.base_from_odom(anchor[0]))) < 0.6 else 10):
                        raise MissionFailed("lost the box")
                center, near = self.base_from_odom(anchor[0]), self.base_from_odom(anchor[1])
                want_near = float(np.linalg.norm(center)) < 0.55
                if want_near != use_near and box is not None:
                    use_near = want_near
                    io.move_arm(near_pose if use_near else far_pose)
                    continue
                u = center / max(float(np.linalg.norm(center)), 1e-6)
                rim = float(near @ u)
                bearing = math.atan2(center[1], max(center[0], 0.05))
                self.say("approach box %d: rim %.3f m ahead along the centre line (goal %.3f), bearing %.0f deg%s" % (
                    step, rim, sx, math.degrees(bearing), " (blind)" if blind else ""))
                if abs(bearing) > 0.25:
                    # Face the box: a drop point to the side lies over a wheel, and the gyro keeps turns accurate.
                    self.turn_by(max(-0.5, min(0.5, bearing)))
                    continue
                if blind and rim > 0.4 and blind < 3:
                    continue                          # far from the box and not seeing it: look again first
                if blind >= 3 and rim > 0.4:
                    self.drive_distance((float(u[0]), float(u[1])), 0.05, speed=0.05, timeout=3.0)   # small nudge
                    continue
                if rim - sx <= 0.03 or (blind and rim - sx <= 0.08):
                    ok = True                      # the arm reaches the drop point from here
                    break
                step_len = min(0.08, rim - sx)
                self.drive_distance((float(u[0]), float(u[1])), step_len, speed=0.06, timeout=3.5)
        finally:
            io.guard.mode = "normal"
            io.stop_base()
        io.wait(0.4)
        if anchor is not None:
            # the placing phase works from this estimate: make it the box memory (odometry-anchored)
            self.box = None
            self.remember_box(anchor[2])
            self.box["center"], self.box["near"] = anchor[0], anchor[1]
        io.move_arm(self.carry_pose(True))
        box = self.see_box(tries=4, expect=None if anchor is None else self.base_from_odom(anchor[1]))
        if box is not None:
            # A fresh look at close range beats the odometry-carried estimate (whose bearing drifts): replace it.
            old_c = self.box_center_base()
            self.say("final look: box centre %s, carried estimate %s" % (
                np.round(box.center_xy, 3), None if old_c is None else np.round(old_c, 3)))
            self.box = None
            self.remember_box(box)
        t = self.box_near_base()
        self.say("near rim in base frame %s (goal x=%.3f)" % (np.round(t, 3), sx))
        if not ok and np.linalg.norm(t - np.array([sx, 0.0])) > 0.04:
            raise MissionFailed("could not reach the box")
        return "PLACE"

    def s_place(self):
        io = self.io
        drop = self.box_target()
        rim = self.box["rim"]
        for attempt in range(1, self.m_cfg["place_retries"] + 1):
            hover = np.array([drop[0], drop[1], rim + 0.11])
            low = np.array([drop[0], drop[1], rim + 0.06])
            self.move_to_tcp(hover, seed=self.carry_pose(True))
            self.move_to_tcp(low, seed=io.arm_command(), duration_ms=1200)
            self.open_gripper()
            io.wait(0.6)
            self.move_to_tcp(hover, seed=io.arm_command())
            io.move_arm(self.carry_pose(False))
            holding = self.holding_cube()
            self.say("still holding after release: %s" % holding)
            if holding is not True:
                self.cube_odom = None
                return "EXTRACT"
        raise MissionFailed("cube did not leave the gripper")

    def inspect_box(self):
        """Raise the camera over the drop point and look for the cube; falls back to the drop point."""
        io = self.io
        drop = self.box_target()
        point = np.array([drop[0], drop[1], self.box_floor() + self.cube_cfg["size_m"] / 2])
        servo, err = self.arm.look_at_ik(point, 0.25, self.search_pose())
        if servo is None:
            self.say("no look-down pose over the drop point; using the carry pose")
            servo = np.array(self.search_pose()[:5], float)
        pose = list(np.round(servo).astype(int)) + [self.arm_cfg["gripper_open"]]
        io.move_arm(pose)
        for _ in range(3):
            look = self.look(settle=0.5)
            for obs in detect_cubes(look.frame, self.cube_cfg, self.cam_cfg):
                base = self.cube_base(look, obs)
                if self.box_contains(base, margin=-0.03):
                    return base
            io.wait(0.3)
        self.say("cube not seen from above; assuming it lies at the drop point")
        return point

    def box_contains(self, base_xyz, margin=0.0):
        center = self.base_from_odom(self.box["center"])
        yaw = self.box["yaw"] - self.io.pose()[2]
        c, s = math.cos(-yaw), math.sin(-yaw)
        d = np.asarray(base_xyz[:2]) - center
        local = np.array([c * d[0] - s * d[1], s * d[0] + c * d[1]])
        return abs(local[0]) < self.box["length"] / 2 - margin and abs(local[1]) < self.box["width"] / 2 - margin

    def s_extract(self):
        io = self.io
        for attempt in range(1, self.m_cfg["extract_retries"] + 1):
            cube = self.inspect_box()
            if cube is None:
                self.say("cube not visible inside the box (attempt %d)" % attempt)
                continue
            self.say("cube in box at %s" % np.round(cube, 3))
            target = np.array([cube[0], cube[1], self.box_floor() + self.cube_cfg["size_m"] / 2])
            self.open_gripper()
            try:
                converged = self.servo_cube(target)
            except MissionFailed as exc:
                self.say("extraction servoing failed: %s" % exc)
                converged = False
            if not converged:
                io.move_arm(self.carry_pose(False))
                if attempt < 2:
                    continue
                # The cube we just dropped cannot be seen from here, but we know where it went: grasp there blind and
                # let the gripper's stall feedback say whether it is held.
                self.say("extraction: grasping blind at the drop point %s" % np.round(target, 3))
                self.open_gripper()
                try:
                    self.move_to_tcp(target + np.array([0, 0, 0.08]), seed=io.arm_command())
                    self.move_to_tcp(target, seed=io.arm_command(), duration_ms=1200)
                except (MissionFailed, ValueError) as exc:
                    self.say("blind grasp not reachable: %s" % exc)
                    io.move_arm(self.carry_pose(False))
                    continue
            self.close_gripper()
            tcp = self.arm.tcp_position(io.arm_command())
            up = np.array([tcp[0], tcp[1], self.box["rim"] + 0.10])
            self.move_to_tcp(up, duration_ms=1500)
            io.move_arm(self.carry_pose(True))
            if self.holding_cube() is not False:
                return "FINISH"
            self.open_gripper()
        raise MissionFailed("could not take the cube out of the box")

    def box_floor(self):
        return 0.01

    def s_finish(self):
        io = self.io
        action = self.m_cfg["final_action"]
        if action == "place_on_floor":
            # Back away from the box, then lower the cube to the floor in front of the robot.
            self.drive_distance((-1.0, 0.0), 0.18)
            floor_target = np.array([0.22, 0.0, self.cube_cfg["size_m"] / 2 + 0.004])
            self.move_to_tcp(floor_target + np.array([0, 0, 0.08]), seed=self.carry_pose(True))
            self.move_to_tcp(floor_target, seed=io.arm_command(), duration_ms=1200)
            self.open_gripper()
            io.wait(0.5)
            self.move_to_tcp(floor_target + np.array([0, 0, 0.10]), seed=io.arm_command())
            io.move_arm(self.carry_pose(False))
        self.result = "cube found, put into the box and taken out"
        return "DONE"
