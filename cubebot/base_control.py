"""Velocity shaping and lidar obstacle guard for the holonomic base."""
import math

import numpy as np

from .geometry import wrap


def _ramp(current, target, limit, dt):
    step = limit * dt
    return current + max(-step, min(step, target - current))


class VelocityShaper:
    """Saturate and acceleration-limit a body twist (vx, vy, wz)."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.out = [0., 0., 0.]

    def step(self, desired, dt):
        c = self.cfg
        lim = (c["max_vx"], c["max_vy"], c["max_wz"])
        target = [max(-m, min(m, v)) for v, m in zip(desired, lim)]
        accel = (c["accel"], c["accel"], c["angular_accel"])
        self.out = [_ramp(o, t, a, dt) for o, t, a in zip(self.out, target, accel)]
        return tuple(self.out)

    def reset(self):
        self.out = [0., 0., 0.]


class ObstacleGuard:
    """Reduces the commanded twist when lidar points lie in the swept footprint."""

    def __init__(self, cfg):
        fp = cfg["footprint"]
        self.front, self.back, self.half = fp["front"], fp["back"], fp["half_width"]
        self.stop_margin = cfg["stop_margin_m"]
        self.slow_margin = cfg["slow_margin_m"]
        self.dock_margin = cfg["dock_stop_margin_m"]
        self.dock_slow = cfg["dock_slow_margin_m"]
        self.mode = "normal"          # "normal" | "dock"
        self.last_block = None

    def _free_distance(self, points, direction, reach):
        """Distance the footprint can slide along `direction` before touching a point."""
        if len(points) == 0:
            return reach
        # A point already inside the footprint and behind the motion cannot be hit by moving away.
        inside0 = ((points[:, 0] > -self.back) & (points[:, 0] < self.front)
                   & (np.abs(points[:, 1]) < self.half))
        # Returns inside the footprint are the robot itself (chassis, lidar mounts, arm base), never obstacles.
        points = points[~inside0]
        if len(points) == 0:
            return reach
        s = np.linspace(0.0, reach, max(2, int(reach / 0.02) + 1))
        # Point positions relative to the shifted footprint, for every shift.
        rel_x = points[:, 0][None, :] - direction[0] * s[:, None]
        rel_y = points[:, 1][None, :] - direction[1] * s[:, None]
        inside = (rel_x > -self.back) & (rel_x < self.front) & (np.abs(rel_y) < self.half)
        hit = inside.any(axis=1)
        if not hit.any():
            return reach
        return float(s[np.argmax(hit)])

    def limit(self, twist, points, dt_horizon=1.2):
        vx, vy, wz = twist
        margin = self.dock_margin if self.mode == "dock" else self.stop_margin
        slow = self.dock_slow if self.mode == "dock" else self.slow_margin
        self.last_block = None
        if points is None or len(points) == 0:
            return twist
        speed = math.hypot(vx, vy)
        if speed > 1e-4:
            direction = (vx / speed, vy / speed)
            reach = slow + speed * dt_horizon
            # Points already inside the footprint inflated by the margin block that direction.
            free = self._free_distance(points, direction, reach) - margin
            scale = max(0.0, min(1.0, free / max(1e-3, slow - margin)))
            if scale < 1.0:
                self.last_block = ("translate", free)
            vx, vy = vx * scale, vy * scale
        if abs(wz) > 1e-4:
            sweep = math.copysign(0.25, wz)
            ok = True
            for frac in (0.5, 1.0):
                a = sweep * frac
                c, s = math.cos(-a), math.sin(-a)
                own = ((points[:, 0] > -self.back) & (points[:, 0] < self.front) & (np.abs(points[:, 1]) < self.half))
                rel = points[~own] @ np.array([[c, s], [-s, c]])      # the robot's own returns never block a turn
                # Rotating the world by -a is the same as rotating the footprint by a.
                inside = ((rel[:, 0] > -self.back) & (rel[:, 0] < self.front)
                          & (np.abs(rel[:, 1]) < self.half))
                if inside.any():
                    ok = False
                    break
            if not ok:
                wz = 0.0
                self.last_block = ("rotate", 0.0)
        return (vx, vy, wz)
