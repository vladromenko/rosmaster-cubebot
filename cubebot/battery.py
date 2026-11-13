"""Battery state of charge from the pack voltage (3S Li-ion/LiPo assumed; there is no current sensor).

Method: a rest-voltage curve, a load correction while the base or arm moves, a slow median/EMA filter, and a
remaining-time estimate fitted to the recent discharge. Every number is an estimate with a stated error band, not a
measurement; the tool logs voltage/time to disk so the curve can be tuned to the real pack later.
"""
import bisect
import csv
import os
import time
from collections import deque

# (rest voltage of the whole 3S pack, percent)
CURVE_3S = [(9.60, 0), (10.20, 5), (10.50, 10), (10.80, 20), (11.10, 35), (11.40, 50), (11.70, 65), (12.00, 80),
            (12.30, 90), (12.60, 100)]


def soc_from_rest_voltage(volts, curve=CURVE_3S):
    xs = [v for v, _ in curve]
    if volts <= xs[0]:
        return 0.0
    if volts >= xs[-1]:
        return 100.0
    i = bisect.bisect_right(xs, volts)
    (v0, p0), (v1, p1) = curve[i - 1], curve[i]
    return p0 + (p1 - p0) * (volts - v0) / (v1 - v0)


class BatteryGauge:
    def __init__(self, log_path="~/.cache/rosmaster/battery_log.csv", sag_moving=0.15, sag_arm=0.10, curve=CURVE_3S):
        self.curve = curve
        self.sag_moving, self.sag_arm = sag_moving, sag_arm
        self.raw = None
        self.volts_filtered = None
        self.history = deque(maxlen=4000)          # (time, soc)
        self._window = deque(maxlen=15)
        self._last_value, self._last_change = None, time.monotonic()
        self.charging = False
        self.log_path = os.path.expanduser(log_path) if log_path else None
        self._next_log = 0.0

    def update(self, volts, base_moving=False, arm_moving=False, now=None):
        """Feed one voltage sample (about 1 Hz); returns the current state dict."""
        now = time.monotonic() if now is None else now
        if volts is None or not 6.0 < volts < 20.0:
            return self.state(now)
        if volts != self._last_value:
            self._last_value, self._last_change = volts, now
        self.raw = volts
        compensated = volts + (self.sag_moving if base_moving else 0.0) + (self.sag_arm if arm_moving else 0.0)
        self._window.append(compensated)
        median = sorted(self._window)[len(self._window) // 2]
        self.volts_filtered = median if self.volts_filtered is None else 0.9 * self.volts_filtered + 0.1 * median
        self.charging = self.volts_filtered > 12.55
        soc = soc_from_rest_voltage(self.volts_filtered, self.curve)
        self.history.append((now, soc))
        self._write_log(now, volts, soc, base_moving, arm_moving)
        return self.state(now)

    def stale(self, now=None, moving=False):
        """The factory watchdog patch can freeze the ADC update while /cmd_vel streams; flag a stuck reading."""
        now = time.monotonic() if now is None else now
        return moving and now - self._last_change > 20.0

    def remaining_minutes(self):
        """Linear fit of SoC over the last ~10 minutes; None when not discharging or too little data."""
        if len(self.history) < 30:
            return None
        t1, s1 = self.history[-1]
        old = [(t, s) for t, s in self.history if t1 - t <= 600]
        if len(old) < 30 or old[-1][0] - old[0][0] < 120:
            return None
        t0, s0 = old[0]
        slope = (s1 - s0) / (t1 - t0)              # percent per second
        if slope >= -1e-5 or self.charging:
            return None
        return max(0.0, (s1 - 0.0) / -slope / 60.0)

    def state(self, now=None):
        soc = None if self.volts_filtered is None else soc_from_rest_voltage(self.volts_filtered, self.curve)
        return dict(volts=self.raw, soc=None if soc is None else round(soc, 1), charging=self.charging,
                    minutes_left=None if self.remaining_minutes() is None else round(self.remaining_minutes()),
                    error_band_percent=8 if soc is not None else None)

    def _write_log(self, now, volts, soc, base_moving, arm_moving):
        if not self.log_path or now < self._next_log:
            return
        self._next_log = now + 10.0
        try:
            os.makedirs(os.path.dirname(self.log_path), exist_ok=True)
            new = not os.path.exists(self.log_path)
            with open(self.log_path, "a", newline="") as handle:
                w = csv.writer(handle)
                if new:
                    w.writerow(["unix_time", "volts", "soc_estimate", "base_moving", "arm_moving"])
                w.writerow([round(time.time(), 1), round(volts, 3), round(soc, 1), int(base_moving), int(arm_moving)])
        except OSError:
            pass
