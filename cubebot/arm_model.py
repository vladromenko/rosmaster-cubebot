"""Rosmaster M3 Pro arm: forward/inverse kinematics from the Explorer URDF.

The firmware never reports servo positions, so every pose here is a *commanded*
estimate. Servo convention (config/arm_model.json in Explorer):
    q1 = radians(90 - s1),  q2..q5 = radians(s - 90)
"""
import itertools
import json
import math

import numpy as np
from scipy.optimize import least_squares

from .geometry import axis_angle, rpy_matrix, transform

# (xyz, rpy, axis) of arm1..arm5 relative to the previous link, from explorer.urdf.
CHAIN = [
    ((0.033617, -1.2421e-05, 0.185), (6.3751e-05, 0., 0.028395), (0., 0., 1.)),
    ((-0.0021522, 6.1185e-05, 0.033), (-6.3725e-05, 0., -0.028422), (0., -1., -6.3725e-05)),
    ((0., 0., 0.08285), (6.3702e-05, 0., 0.027102), (-0.027098, -0.99963, 0.)),
    ((0., 0., 0.08285), (0., 0., -0.027075), (-2.6392e-05, -1., 0.)),
    ((0.00057305, 0., 0.074815), (0., 0., 0.), (0., 0., 1.)),
]
# DCW2 camera link is fixed to arm4 (the wrist-pitch link), so J5 roll does not move it.
DCW2_ON_ARM4 = ((-0.0483883323542816, 0.00245180097510241, 0.0671900000306381),
                (1.84287098025439, -1.57079631189374, -1.81447538896515))

SERVO_LIMITS = [(0, 180), (0, 180), (0, 180), (0, 180), (0, 270), (30, 180)]
SHOULDER_HEIGHT = 0.218
CHASSIS_BOX = ((-0.155, 0.13), (-0.12, 0.12), (0.0, 0.14))   # base_link mesh: x -0.153..0.124, y +-0.11


def optical_candidates():
    """The 24 proper rotations that map camera-link axes onto optical axes."""
    out = []
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            r = np.zeros((3, 3))
            for row, (col, sign) in enumerate(zip(perm, signs)):
                r[row, col] = sign
            if np.linalg.det(r) > 0:
                out.append(r.T)  # columns = optical axes expressed in link frame
    return out


def servo_to_q(servo):
    s = np.asarray(servo, float)
    q = np.radians(s[:5] - 90.0)
    q[0] = math.radians(90.0 - s[0])
    return q


def q_to_servo(q):
    q = np.asarray(q, float)
    s = np.degrees(q) + 90.0
    s[0] = 90.0 - math.degrees(q[0])
    return s


# REP-103 body->optical (x right, y down, z forward) for a link whose x points forward.
# It equals the vendor EndToCamMat rotation, and the vendor FK of its own "init" pose
# (pitch 45 deg, tcp 0.160/0.216 m) matches this chain, which confirms the servo signs.
REP103_OPTICAL = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])


class ArmModel:
    def __init__(self, tcp_offset=0.085, optical_index=None, optical_rotation=None):
        self.tcp_offset = tcp_offset
        if optical_rotation is not None:
            self.optical = np.asarray(optical_rotation, float)
        elif optical_index is not None:
            self.optical = optical_candidates()[optical_index]
        else:
            self.optical = REP103_OPTICAL
        self._cam_link = transform(DCW2_ON_ARM4[0], rpy_matrix(*DCW2_ON_ARM4[1]))

    def fk(self, servo):
        q = servo_to_q(servo)
        t = np.eye(4)
        frames = []
        for (xyz, rpy, axis), angle in zip(CHAIN, q):
            t = t @ transform(xyz, rpy_matrix(*rpy)) @ transform(rotation=axis_angle(axis, angle))
            frames.append(t)
        arm4, arm5 = frames[3], frames[4]
        tcp = arm5 @ transform((0., 0., self.tcp_offset))
        cam = arm4 @ self._cam_link
        cam_opt = cam.copy()
        cam_opt[:3, :3] = cam[:3, :3] @ self.optical
        return {"arm4": arm4, "arm5": arm5, "tcp": tcp, "camera": cam_opt, "frames": frames}

    def tcp_position(self, servo):
        return self.fk(servo)["tcp"][:3, 3]

    def approach_pitch_down(self, servo):
        a = self.fk(servo)["arm5"][:3, 2]
        return math.asin(max(-1., min(1., -a[2])))

    def camera_pose(self, servo):
        return self.fk(servo)["camera"]

    def tcp_in_camera(self, servo):
        f = self.fk(servo)
        return np.linalg.inv(f["camera"]) @ f["tcp"] @ np.array([0., 0., 0., 1.])

    def link_points(self, servo, samples=6):
        """Points along the arm for collision checks (base frame)."""
        f = self.fk(servo)
        pts = [np.array([0.033617, 0., 0.185])]
        prev = pts[0]
        for frame in f["frames"] + [f["tcp"]]:
            cur = frame[:3, 3]
            for k in range(1, samples + 1):
                pts.append(prev + (cur - prev) * k / samples)
            prev = cur
        return np.array(pts)

    def collision_free(self, servo, floor_margin=0.012, allow_inside_box=None):
        pts = self.link_points(servo)
        if np.any(pts[:, 2] < floor_margin):
            return False
        # Anything below chassis top height must stay outside the chassis footprint.
        (x0, x1), (y0, y1), (z0, z1) = CHASSIS_BOX
        inside = ((pts[:, 0] > x0) & (pts[:, 0] < x1) & (pts[:, 1] > y0) & (pts[:, 1] < y1)
                  & (pts[:, 2] > z0) & (pts[:, 2] < z1))
        return not np.any(inside)

    def within_limits(self, servo):
        return all(lo - 1e-6 <= v <= hi + 1e-6 for v, (lo, hi) in zip(servo, SERVO_LIMITS))

    def ik(self, target, pitch_down_deg, seed, pitch_weight=0.15, roll_servo=90.0, stay_weight=0.0, quick=False, max_step_deg=None):
        """Servos [s1..s6-less] placing the TCP at `target` (base frame).

        Returns (servo[5], residual_m) or (None, residual) if nothing feasible.
        Position error dominates; the approach pitch is a soft preference so the
        solver may deviate when the pose is only reachable with another tilt.
        """
        target = np.asarray(target, float)
        pitch = math.radians(pitch_down_deg)
        lo = np.array([SERVO_LIMITS[i][0] for i in range(4)], float)
        hi = np.array([SERVO_LIMITS[i][1] for i in range(4)], float)

        def residual(x):
            servo = np.concatenate([x, [roll_servo, 90.0]])
            f = self.fk(servo)
            err = f["tcp"][:3, 3] - target
            a = f["arm5"][:3, 2]
            pitch_err = math.asin(max(-1., min(1., -a[2]))) - pitch
            stay = stay_weight * (x - seed4) if stay_weight else np.zeros(0)
            return np.concatenate([err, [pitch_weight * pitch_err], stay])

        seed4 = np.asarray(seed[:4], float)
        if max_step_deg is not None:
            if not math.isfinite(max_step_deg) or max_step_deg <= 0:
                raise ValueError("Positive local joint step required")
            lo = np.maximum(lo, seed4 - max_step_deg)
            hi = np.minimum(hi, seed4 + max_step_deg)
        seeds = [seed4.copy()]
        for dq2, dq3 in (() if quick else itertools.product((-30, 0, 30), (-40, 0, 40))):
            s = seeds[0].copy()
            s[1] += dq2
            s[2] += dq3
            seeds.append(np.clip(s, lo + 1, hi - 1))
        # Base yaw straight from geometry makes every seed converge faster.
        dx = target[0] - CHAIN[0][0][0]
        dy = target[1] - CHAIN[0][0][1]
        yaw = math.degrees(math.atan2(dy, dx))
        best = None
        for s in seeds:
            s = s.copy()
            if not quick:
                s[0] = 90.0 - yaw
            res = least_squares(residual, np.clip(s, lo, hi), bounds=(lo, hi), xtol=1e-6, ftol=1e-6)
            pos_err = float(np.linalg.norm(res.fun[:3]))
            servo = np.concatenate([res.x, [roll_servo, 90.0]])
            cost = pos_err + (1.0 if not self.collision_free(servo) else 0.0)
            if stay_weight:
                cost += 0.2 * float(np.linalg.norm(res.x - seed4)) * stay_weight
            if best is None or cost < best[2]:
                best = (res.x.copy(), pos_err if cost < 1.0 else cost, cost)
        if best is None or best[1] > 0.01:
            return None, (best[1] if best else float("inf"))
        return np.concatenate([best[0], [roll_servo]]), best[1]

    def camera_jacobian_deg(self, servo, point_base, step_deg=1.0):
        """d(point in camera frame)/d(servo J1..J4 in degrees), 3x4, for visual servoing."""
        base = np.linalg.inv(self.camera_pose(servo)) @ np.append(point_base, 1.0)
        jac = np.zeros((3, 4))
        for i in range(4):
            s = np.array(servo, float)
            s[i] += step_deg
            moved = np.linalg.inv(self.camera_pose(s)) @ np.append(point_base, 1.0)
            jac[:, i] = (moved[:3] - base[:3]) / step_deg
        return jac

    def tcp_jacobian_deg(self, servo, step_deg=1.0):
        """d(TCP in base)/d(servo J1..J4 in degrees), 3x4."""
        p0 = self.tcp_position(servo)
        jac = np.zeros((3, 4))
        for i in range(4):
            s = np.array(servo, float)
            s[i] += step_deg
            jac[:, i] = (self.tcp_position(s) - p0) / step_deg
        return jac

    PITCH_PREFERENCE = (70, 80, 60, 90, 50, 40, 30, 20)

    def grasp_ik(self, target, seed, pitches=None, roll_servo=90.0):
        """First feasible IK over the preferred approach pitches: (servo[5], pitch) or (None, None)."""
        for pitch in pitches or self.PITCH_PREFERENCE:
            sol, err = self.ik(target, pitch, seed, roll_servo=roll_servo)
            if sol is not None:
                return sol, pitch
        return None, None

    def cache_key(self, z, seed):
        return "z=%.4f;tcp=%.4f;seed=%s" % (z, self.tcp_offset, ",".join("%d" % v for v in seed))

    def reach_window(self, z, seed, y=0.0, x_range=(0.08, 0.36), step=0.01, cache_files=()):
        """(x_min, x_max) of floor-level targets (relative to the base origin) the arm can grasp.

        Scanning IK is slow on the robot (~1.5 min), so results are cached in the JSON files
        `cache_files` (first existing hit wins; the first writable one receives new entries).
        """
        key = self.cache_key(z, seed)
        for path in cache_files:
            try:
                with open(path) as handle:
                    hit = json.load(handle).get(key)
                if hit is not None:
                    return tuple(hit) if hit else None
            except (OSError, ValueError):
                continue
        result = self._scan_window(z, seed, y, x_range, step)
        for path in cache_files:
            try:
                try:
                    with open(path) as handle:
                        data = json.load(handle)
                except (OSError, ValueError):
                    data = {}
                data[key] = list(result) if result else []
                with open(path, "w") as handle:
                    json.dump(data, handle, indent=1)
                break
            except OSError:
                continue
        return result

    def _scan_window(self, z, seed, y, x_range, step):
        feasible = []
        for x in np.arange(x_range[0], x_range[1] + 1e-9, step):
            sol, _ = self.grasp_ik([x, y, z], seed)
            if sol is not None:
                feasible.append(float(x))
        if not feasible:
            return None
        return feasible[0], feasible[-1]

    def pitch_jacobian_deg(self, servo, step_deg=1.0):
        """d(approach pitch-down, radians)/d(servo J1..J4 in degrees)."""
        p0 = self.approach_pitch_down(servo)
        out = np.zeros(4)
        for i in range(4):
            s = np.array(servo, float)
            s[i] += step_deg
            out[i] = (self.approach_pitch_down(s) - p0) / step_deg
        return out

    def look_at_ik(self, point, distance, seed, min_pitch_deg=35.0):
        """Servo J1..J4 that place the camera `distance` metres from `point`, looking at it."""
        point = np.asarray(point, float)
        lo = np.array([SERVO_LIMITS[i][0] for i in range(4)], float)
        hi = np.array([SERVO_LIMITS[i][1] for i in range(4)], float)
        best = None
        for dq2, dq3 in itertools.product((-40, 0, 40), (-40, 0, 40)):
            s = np.array(seed[:4], float)
            s[1] += dq2
            s[2] += dq3

            def residual(x):
                full = np.concatenate([x, [90.0, 90.0]])
                cam = np.linalg.inv(self.fk(full)["camera"]) @ np.append(point, 1.0)
                axis_down = -self.fk(full)["camera"][2, 2]
                too_flat = max(0.0, math.sin(math.radians(min_pitch_deg)) - axis_down)
                return np.concatenate([cam[:3] - [0., 0., distance], [0.5 * too_flat]])

            res = least_squares(residual, np.clip(s, lo, hi), bounds=(lo, hi))
            full = np.concatenate([res.x, [90.0, 90.0]])
            err = float(np.linalg.norm(res.fun[:3]))
            if not self.collision_free(full):
                err += 1.0
            if best is None or err < best[1]:
                best = (res.x.copy(), err)
        return (np.concatenate([best[0], [90.0]]), best[1]) if best[1] < 0.03 else (None, best[1])
