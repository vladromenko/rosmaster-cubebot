"""Planar odometry: firmware wheel pose + IMU gyro + lidar scan matching.

Frames: `odom` starts at the pose where CubeBot starts; `base` is base_footprint
(x forward, y left). The fused pose is the base pose in `odom`.

* Wheels: increments of the firmware-integrated /odom_raw pose, in the body frame.
  Mecanum wheels slip sideways and while turning, so wheel yaw is the weakest axis.
* Gyro: bias-compensated z rate integrated between samples; it replaces most of the
  wheel yaw increment.
* Lidar: each new merged scan is registered against the previous one with
  point-to-line ICP seeded by the wheel/gyro increment. The ICP result replaces the
  prediction only when it is well constrained and close to it.
"""
import math
import threading

from .geometry import se2_between, se2_compose, wrap
from .scan_matching import icp


class FusedOdometry:
    def __init__(self, cfg):
        self.cfg = cfg
        self._lock = threading.Lock()
        self.pose = (0., 0., 0.)        # fused pose at the last scan
        self._wheel = (0., 0., 0.)      # wheel increment since the last scan (body frame)
        self._gyro_yaw = 0.             # gyro yaw increment since the last scan
        self._wheel_prev = None
        self._gyro_t = None
        self._gyro_bias = 0.
        self._bias_samples = []
        self._prev_scan = None
        self.stationary = True
        self.scan_matching_enabled = True
        self.stats = dict(icp_used=0, icp_rejected=0, icp_degenerate=0, last_rms=None, last_ratio=None)

    # -- inputs -----------------------------------------------------------------
    def on_wheel(self, x, y, yaw):
        """Firmware /odom_raw pose (in its own frame)."""
        with self._lock:
            pose = (x, y, yaw)
            if self._wheel_prev is not None:
                d = se2_between(self._wheel_prev, pose)
                if abs(d[0]) < 0.5 and abs(d[1]) < 0.5 and abs(d[2]) < 1.0:  # ignore firmware resets
                    self._wheel = se2_compose(self._wheel, d)
            self._wheel_prev = pose

    def on_gyro(self, t, wz):
        with self._lock:
            if self.stationary:
                self._bias_samples.append(wz)
                del self._bias_samples[:-200]
                if len(self._bias_samples) >= 25:
                    self._gyro_bias = sum(self._bias_samples) / len(self._bias_samples)
            elif self._gyro_t is not None:
                dt = t - self._gyro_t
                if 0 < dt < 0.5:
                    self._gyro_yaw += (wz - self._gyro_bias) * self.cfg["gyro_scale"] * dt
            self._gyro_t = t

    def set_stationary(self, flag):
        """Call with True whenever the base is commanded to stand still (bias learning)."""
        with self._lock:
            if flag and not self.stationary:
                self._bias_samples = []
            self.stationary = flag

    def on_scan(self, scan):
        """Fold the motion since the previous scan into the pose, refined by ICP."""
        with self._lock:
            predicted = self._increment()
            prev, self._prev_scan = self._prev_scan, scan
            delta = predicted
            cfg = self.cfg
            at_rest = abs(predicted[0]) < 0.002 and abs(predicted[1]) < 0.002 and abs(predicted[2]) < 0.002
            if at_rest:
                delta = (0., 0., 0.)           # scan noise must not random-walk a robot that is standing still
            elif prev is not None and cfg["lidar_correction"] and self.scan_matching_enabled:
                result = icp(prev, scan, initial=predicted)
                if result is not None:
                    self.stats["last_rms"], self.stats["last_ratio"] = result.rms, result.inlier_ratio
                    near = (math.hypot(result.delta[0] - predicted[0], result.delta[1] - predicted[1])
                            < cfg["icp_max_correction_m"]
                            and abs(wrap(result.delta[2] - predicted[2])) < cfg["icp_max_correction_rad"])
                    if (not result.degenerate and result.rms < cfg["icp_max_rms_m"]
                            and result.inlier_ratio > cfg["icp_min_inlier_ratio"] and near):
                        delta = result.delta
                        self.stats["icp_used"] += 1
                    else:
                        self.stats["icp_rejected"] += 1
                        self.stats["icp_degenerate"] += int(result.degenerate)
                else:
                    self.stats["icp_rejected"] += 1
            self.pose = se2_compose(self.pose, delta)
            self._wheel = (0., 0., 0.)
            self._gyro_yaw = 0.

    def _increment(self):
        """Wheel translation with the yaw blended towards the gyro."""
        w = self.cfg["gyro_weight"]
        yaw = w * self._gyro_yaw + (1 - w) * self._wheel[2]
        half = (yaw - self._wheel[2]) / 2
        c, s = math.cos(half), math.sin(half)
        return (c * self._wheel[0] - s * self._wheel[1], s * self._wheel[0] + c * self._wheel[1], yaw)

    # -- output -----------------------------------------------------------------
    def current(self):
        """Fused base pose in `odom`, including motion since the last scan."""
        with self._lock:
            return se2_compose(self.pose, self._increment())

    def reset(self):
        with self._lock:
            self.pose = (0., 0., 0.)
            self._wheel = (0., 0., 0.)
            self._gyro_yaw = 0.
            self._prev_scan = None
