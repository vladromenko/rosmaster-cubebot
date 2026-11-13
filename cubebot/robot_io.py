"""Hardware-independent robot interface used by the mission.

Subclasses supply time, sensors and the two actuator publishers (`_publish_twist`,
`_publish_arm`); everything safety related lives here so the real robot and the
simulator are guarded identically.
"""
import math
import os

import numpy as np

from .base_control import ObstacleGuard, VelocityShaper
from .odometry import FusedOdometry


class MissionAbort(Exception):
    pass


class BaseIO:
    TICK = 0.05
    REALTIME = False      # True when `now()` is the wall clock (heavy vision may then run in a thread)

    def __init__(self, cfg, arm_model, log=print):
        self.cfg = cfg
        self.arm = arm_model
        self.log = log
        self.odometry = FusedOdometry(cfg["odometry"])
        self.shaper = VelocityShaper(cfg["base"])
        self.guard = ObstacleGuard(cfg["base"])
        self._desired = (0., 0., 0.)
        self._desired_until = 0.
        self._still_since = None
        self._arm_cmd = [float(v) for v in cfg["arm"]["safe_pose"]]
        self._arm_busy_until = 0.
        self.scan = None
        self.started = None
        self.abort_reason = None
        self.last_output = (0., 0., 0.)
        self.guard_off = False        # operator override in the manual console only

    # --- to be provided by subclasses -----------------------------------------
    def now(self):
        raise NotImplementedError

    def _advance(self, dt):
        raise NotImplementedError

    def _publish_twist(self, vx, vy, wz):
        raise NotImplementedError

    def _publish_arm(self, servo, duration_ms):
        raise NotImplementedError

    def _latest_frame(self):
        raise NotImplementedError

    def battery(self):
        return None

    # --- sensing ----------------------------------------------------------------
    def pose(self):
        return self.odometry.current()

    def arm_command(self):
        return list(self._arm_cmd)

    def measured_arm(self, max_age=1.0):
        """Measured servo angles in degrees (all six), or None when the controller reports none/stale values."""
        return None

    def sync_arm_estimate(self, wait=3.0):
        """Start from where the arm really is instead of assuming a pose; returns the measured angles or None."""
        end = self.now() + wait
        while self.now() < end:
            m = self.measured_arm()
            if m is not None:
                self._arm_cmd = [float(v) for v in m]
                self.log("arm estimate taken from servo feedback: %s" % [round(v) for v in m])
                return m
            self._advance(0.1)
        self.log("no servo feedback: keeping the assumed arm pose %s" % self._arm_cmd)
        return None

    def scan_points(self):
        return None if self.scan is None else self.scan.points

    def frame_after(self, t, timeout=2.0):
        """First camera frame captured after time `t`, or None."""
        end = self.now() + timeout
        while self.now() < end:
            frame = self._latest_frame()
            if frame is not None and frame.stamp > t:
                return frame
            self.wait(self.TICK)
        return None

    # --- base --------------------------------------------------------------------
    def drive(self, vx, vy, wz, hold=0.3, manual=False):
        self.shaper.cfg = dict(self.cfg["base"], **self.cfg.get("manual_base", {})) if manual else self.cfg["base"]
        self._desired = (float(vx), float(vy), float(wz))
        self._desired_until = self.now() + hold

    def stop_base(self):
        self._desired = (0., 0., 0.)
        self.shaper.reset()
        self.last_output = (0., 0., 0.)
        self._publish_twist(0., 0., 0.)

    def control_step(self, dt):
        """One control cycle: deadman, guard, shaping, publish. Safe to call at 20 Hz."""
        desired = self._desired if self.now() < self._desired_until else (0., 0., 0.)
        if self.abort_reason:
            desired = (0., 0., 0.)
        points = self.scan.points if self.scan is not None else None
        guarded = desired if self.guard_off else self.guard.limit(desired, points)
        out = self.shaper.step(guarded, dt)
        if guarded == (0., 0., 0.):
            out = tuple(0. if abs(v) < 0.01 else v for v in out)
            self.shaper.out = list(out)
        self.last_output = out
        self._publish_twist(*out)
        moving = any(abs(v) > 1e-3 for v in out)
        now = self.now()
        if moving:
            self._still_since = None
            self.odometry.set_stationary(False)
        else:
            self._still_since = self._still_since or now
            if now - self._still_since > 0.4:
                self.odometry.set_stationary(True)

    # --- arm ---------------------------------------------------------------------
    def move_arm(self, servo, duration_ms=None, wait=True, check=True):
        servo = [int(round(v)) for v in servo]
        if len(servo) != 6:
            raise ValueError("six servo angles required")
        if not self.arm.within_limits(servo):
            raise ValueError("servo target outside limits: %s" % servo)
        arm_cfg = self.cfg["arm"]
        if check:
            start, end = np.array(self._arm_cmd, float), np.array(servo, float)
            for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
                if not self.arm.collision_free(start + (end - start) * frac, arm_cfg["floor_margin_m"]):
                    raise ValueError("arm path collides with floor or chassis: %s" % servo)
        delta = max(abs(a - b) for a, b in zip(servo, self._arm_cmd))
        if duration_ms is None:
            duration_ms = max(arm_cfg["move_ms_min"], delta * arm_cfg["move_ms_per_deg"])
        duration_ms = int(min(duration_ms, 6000))
        self._publish_arm(servo, duration_ms)
        self._arm_cmd = [float(v) for v in servo]
        self._arm_busy_until = self.now() + duration_ms / 1000.0 + 0.3
        if wait:
            self.wait_arm()

    def wait_arm(self):
        self.wait(max(0.0, self._arm_busy_until - self.now()))

    def arm_idle_since(self):
        return self._arm_busy_until

    # --- timing and safety ---------------------------------------------------------
    def wait(self, seconds):
        end = self.now() + seconds
        while True:
            self._check_abort()
            remaining = end - self.now()
            if remaining <= 1e-9:
                break
            self._advance(min(self.TICK, remaining))
        self._check_abort()

    def request_abort(self, reason):
        self.abort_reason = self.abort_reason or reason

    def _check_abort(self):
        m = self.cfg["mission"]
        if self.started is None:
            self.started = self.now()
        if os.path.exists(m["stop_file"]):
            self.request_abort("stop file present")
        v = self.battery()
        if v is not None and 0 < v < m["min_battery_v"]:
            self.request_abort("battery %.2f V below %.2f V" % (v, m["min_battery_v"]))
        if self.now() - self.started > m["max_runtime_s"]:
            self.request_abort("runtime limit")
        if self.abort_reason:
            self.stop_base()
            raise MissionAbort(self.abort_reason)

    def drive_until(self, command_fn, done_fn, timeout, label=""):
        """Run `command_fn() -> (vx, vy, wz)` each cycle until `done_fn()`; returns success."""
        end = self.now() + timeout
        while self.now() < end:
            if done_fn():
                self.stop_base()
                return True
            self.drive(*command_fn())
            self.wait(self.TICK)
        self.stop_base()
        self.log("timeout: %s" % label)
        return False

    def shutdown(self):
        try:
            self.stop_base()
        finally:
            for _ in range(5):
                self._publish_twist(0., 0., 0.)
