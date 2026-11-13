"""Smooth Cartesian jogging of the arm: the operator moves the gripper tip, IK turns that into servo angles.

Servos are command-only (no readback), so the controller keeps its own state: a target TCP position/pitch that
integrates velocity requests through an acceleration limit, IK with a "stay close" preference, a per-tick joint step
limit and the same floor/chassis collision test the mission uses. Each tick sends one absolute target with a short
servo run time so the servos interpolate; nothing is ever sent that failed a check.
"""
import math

import numpy as np

from .arm_model import SERVO_LIMITS


class CartesianJog:
    WORKSPACE = ((0.08, 0.36), (-0.30, 0.30), (0.025, 0.42))     # x, y, z of the TCP in the base frame (m)

    def __init__(self, arm, cfg, initial_servo):
        self.arm, self.cfg = arm, cfg
        a = cfg["arm"]
        self.open_value, self.close_value = a["gripper_open"], a["gripper_close"]
        self.max_speed = np.array([0.10, 0.10, 0.08])               # m/s at full deflection
        self.accel = 0.6                                           # m/s^2
        self.max_joint_deg_s = 60.0
        self.run_ms = 160
        self.velocity = np.zeros(3)
        self.servo = np.array(initial_servo, float)
        self.pitch = math.degrees(arm.approach_pitch_down(self.servo))
        self.target = arm.tcp_position(self.servo).copy()
        self.roll = float(self.servo[4])
        self.grip = float(self.servo[5])
        self.blocked = None

    def reset(self, servo):
        self.servo = np.array(servo, float)
        self.target = self.arm.tcp_position(self.servo).copy()
        self.pitch = math.degrees(self.arm.approach_pitch_down(self.servo))
        self.roll, self.grip = float(self.servo[4]), float(self.servo[5])
        self.velocity[:] = 0
        self.blocked = None

    def step(self, dt, move, pitch_rate=0.0, roll_rate=0.0, grip_rate=0.0, slow=False):
        """`move` = (x, y, z) deflection in -1..1. Returns the new servo command (list of 6 ints) or None if unchanged."""
        if not math.isfinite(dt) or not 0 < dt <= .15:
            self.velocity[:] = 0
            self.blocked = "control timing; release and retry"
            return None
        if len(move) != 3 or not all(math.isfinite(v) for v in [*move, pitch_rate, roll_rate, grip_rate]):
            raise ValueError("Finite manual input required")
        scale = 0.35 if slow else 1.0
        want = np.clip(np.asarray(move, float), -1, 1) * self.max_speed * scale
        delta = want - self.velocity
        limit = self.accel * dt
        norm = np.linalg.norm(delta)
        if norm > limit:
            delta *= limit / norm
        self.velocity += delta
        candidate = self.target + self.velocity * dt
        for i, (lo, hi) in enumerate(self.WORKSPACE):
            # a pose outside the nominal box (e.g. the upright start pose) may stay or move back in, never further out
            lo, hi = min(lo, self.target[i]), max(hi, self.target[i])
            candidate[i] = min(max(candidate[i], lo), hi)
        pitch = min(max(self.pitch + pitch_rate * dt * 40.0 * scale, min(0.0, self.pitch)), max(90.0, self.pitch))
        roll = min(max(self.roll + roll_rate * dt * 60.0 * scale, SERVO_LIMITS[4][0]), SERVO_LIMITS[4][1])
        grip = min(max(self.grip + grip_rate * dt * 90.0 * scale, self.open_value), self.close_value)
        moved = (np.linalg.norm(candidate - self.target) > 1e-6 or pitch != self.pitch or roll != self.roll
                 or grip != self.grip)
        if not moved:
            if np.any(want):
                self.blocked = "workspace limit; try joints mode"
                self.velocity[:] = 0
            return None
        self.blocked = None
        cartesian = np.linalg.norm(candidate - self.target) > 1e-6 or pitch != self.pitch
        if cartesian:
            sol, err = self.arm.ik(candidate, pitch, self.servo, stay_weight=0.0006,
                                   quick=True, roll_servo=roll, max_step_deg=self.max_joint_deg_s * dt)
            if sol is None:
                self.blocked = "unreachable; try joints mode"
                self.velocity[:] = 0
                return None
            new = np.concatenate([sol[:4], [roll, grip]])
            actual = self.arm.tcp_position(new)
            requested = candidate - self.target
            if np.linalg.norm(requested) > 1e-6 and np.dot(actual - self.target, requested) <= 1e-10:
                self.blocked = "workspace/singularity; try joints mode"
                self.velocity[:] = 0
                return None
        else:
            # Gripper and wrist roll must not invoke IK or move shoulder/elbow.
            new = np.concatenate([self.servo[:4], [roll, grip]])
        rounded = np.round(new)
        if not self.arm.within_limits(rounded) or not self.arm.collision_free(rounded, self.cfg["arm"]["floor_margin_m"]):
            self.blocked = "collision"
            self.velocity[:] = 0
            return None
        previous = np.round(self.servo)
        # Keep the next Cartesian target at the solved pose; never integrate unreachable motion.
        self.target = self.arm.tcp_position(new)
        self.pitch = math.degrees(self.arm.approach_pitch_down(new))
        self.roll, self.grip, self.servo = roll, grip, new
        if np.array_equal(rounded, previous):
            return None
        return [int(v) for v in rounded]

    def step_joints(self, dt, rates, slow=False):
        """Direct joint jog uses the same limits/collision model and output transport."""
        if not math.isfinite(dt) or not 0 < dt <= .15:
            self.blocked = "control timing; release and retry"
            return None
        values = np.asarray(rates, float)
        if values.shape != (6,) or not np.isfinite(values).all():
            raise ValueError("Six finite joint intents required")
        speed = np.array([30., 30., 30., 30., 40., 60.]) * (.35 if slow else 1.)
        goal = self.servo + np.clip(values, -1., 1.) * speed * dt
        limits = list(SERVO_LIMITS[:5]) + [(self.open_value, self.close_value)]
        goal = np.clip(goal, [v[0] for v in limits], [v[1] for v in limits])
        rounded = np.round(goal)
        if not self.arm.within_limits(rounded) or not all(self.arm.collision_free(
                self.servo + (rounded - self.servo) * t, self.cfg["arm"]["floor_margin_m"])
                for t in (0., .25, .5, .75, 1.)):
            self.blocked = "collision"
            return None
        changed = not np.array_equal(rounded, np.round(self.servo))
        self.reset(goal)
        return [int(v) for v in rounded] if changed else None
