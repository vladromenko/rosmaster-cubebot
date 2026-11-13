"""Mirror an SO101 leader arm (LeRobot) onto the M3 arm, joint by joint.

Both arms have the same joint order (pan, lift, elbow, wrist flex, wrist roll, gripper), so the mapping is
    servo_i = 90 + sign_i * scale_i * (leader_deg_i - neutral_i)         (gripper: 0..100 -> open..close)
Signs/neutral come from config/mirror.json and MUST be confirmed with small motions first (scale starts at 0.5).
The mapper never sends a pose that fails the arm's limit/collision test and limits every joint's speed.
"""
import json
import math
from pathlib import Path

import numpy as np

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
DEFAULTS = dict(
    sign=[1, -1, 1, 1, 1],
    scale=[0.5, 0.5, 0.5, 0.5, 0.5],
    neutral=[0, 0, 0, 0, 0],
    max_deg_s=70.0,
    timeout_s=0.5,
    smoothing=0.35,
)


def load(path=None):
    cfg = dict(DEFAULTS)
    p = Path(path) if path else Path(__file__).resolve().parent.parent / "config" / "mirror.json"
    if p.exists():
        cfg.update(json.loads(p.read_text()))
    return cfg


class MirrorMapper:
    def __init__(self, arm, arm_cfg, mirror_cfg, start_servo):
        self.arm, self.a, self.m = arm, arm_cfg, mirror_cfg
        self.target = np.array(start_servo, float)      # where the leader says the follower should be
        self.servo = np.array(start_servo, float)       # what we last commanded
        self.last_msg = -1e9
        self.blocked = None
        self.ref = None                                  # (leader deg, servo) captured at the first frame in relative mode

    def update(self, leader, now):
        """`leader`: dict joint name -> value (degrees; gripper 0..100)."""
        try:
            deg = [float(leader[name]) for name in JOINTS[:5]]
            grip = float(leader["gripper"])
        except (KeyError, TypeError, ValueError):
            return False
        if not all(math.isfinite(v) for v in deg + [grip]):
            return False
        t = self.target.copy()
        if self.m.get("relative") and self.ref is None:
            self.ref = (list(deg), self.servo[:5].copy(), grip, float(self.servo[5]))
        for i in range(5):
            if self.ref is not None:
                t[i] = self.ref[1][i] + self.m["sign"][i] * self.m["scale"][i] * (deg[i] - self.ref[0][i])
                continue
            t[i] = 90.0 + self.m["sign"][i] * self.m["scale"][i] * (deg[i] - self.m["neutral"][i])
        t[5] = self.a["gripper_open"] + (self.a["gripper_close"] - self.a["gripper_open"]) * min(max(grip, 0.0), 100.0) / 100.0
        if self.ref is not None and self.m.get("relative_gripper", True):
            t[5] = self.ref[3] + (grip - self.ref[2]) * (self.a["gripper_close"] - self.a["gripper_open"]) / 100.
            t[5] = min(max(t[5], self.a["gripper_open"]), self.a["gripper_close"])
        s = self.m["smoothing"]
        self.target = (1 - s) * self.target + s * t
        self.last_msg = now
        return True

    def rebase(self):
        """Forget the reference: the next leader frame becomes the zero point (call when mirroring starts)."""
        self.ref = None

    def step(self, dt, now):
        """Next servo command (list of 6 ints) or None (no data, timed out, or nothing to change)."""
        if now - self.last_msg > self.m["timeout_s"]:
            self.blocked = "no leader data"
            return None
        limit = self.m["max_deg_s"] * dt + 0.5
        goal = np.clip(self.target, [lo for lo, _ in self._limits()], [hi for _, hi in self._limits()])
        move = np.clip(goal - self.servo, -limit, limit)
        new = np.round(self.servo + move)
        if np.array_equal(new, np.round(self.servo)):
            return None
        if not self.arm.within_limits(new) or not self.arm.collision_free(new, self.a["floor_margin_m"]):
            self.blocked = "limit/collision"
            return None
        self.blocked = None
        self.servo = new
        return [int(v) for v in new]

    @staticmethod
    def _limits():
        from .arm_model import SERVO_LIMITS
        return SERVO_LIMITS
