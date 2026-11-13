"""Bridge core: turns the native controller protocol into the stock robotio interface. No ROS imports.

Topics produced (via `publish`): odom, imu, mag, battery, scan0, scan1, arm_servo (measured servo degrees).
Commands consumed: cmd_vel, arm (servo degrees + run time), beep. The controller needs a session, leases of at most
250 ms and an uploaded calibration; this class keeps all of that alive, so callers just stream the latest command.
"""
import json
import math
import os
import struct
import time

from . import link as L
from .decode import ScanAssembler, decode_status

LEASE_US = 240_000


def servo_to_q(joint, deg):
    """Servo command degrees -> calibrated joint radians (Explorer arm_model.json convention)."""
    if joint == 1:
        return -math.radians(deg - 90.0)
    if joint == 6:
        return math.radians(deg)
    return math.radians(deg - 90.0)


def q_to_servo(joint, q):
    if joint == 1:
        return 90.0 - math.degrees(q)
    if joint == 6:
        return math.degrees(q)
    return math.degrees(q) + 90.0


class BridgeCore:
    def __init__(self, link, calibration, publish, log=print, clock=time.monotonic):
        self.link, self.cal, self.publish, self.log, self.clock = link, calibration, publish, log, clock
        self.identity = None
        self.state = {}
        self.last_status = None
        self.phase = "searching"          # searching -> opening -> calibrating -> ready ; fault -> clearing
        self.phase_since = clock()
        self.pending = {}                 # sequence -> (op, time)
        self.hello_sent = {}
        self.last_hello = 0.0
        self.ignore_fault_until = 0.0
        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_time = -1e9
        self.arm_target = None            # (radians[6], runtime_ms, time)
        self.arm_dirty = False
        self.last_arm_send = 0.0
        self.last_arm_enable = 0.0
        self.last_base_send = 0.0
        self.pose = [0.0, 0.0, 0.0]
        self.last_odom_us = None
        self.scans = [ScanAssembler(), ScanAssembler()]
        self.servo_deg = [None] * 6
        self.stats = dict(frames=0, results_rejected=0, boots=0)
        self.frames = []

    # --- inputs ---------------------------------------------------------------------------
    def on_frame(self, kind, payload):
        self.frames.append((kind, payload))

    def on_cmd_vel(self, vx, vy, wz):
        self.cmd, self.cmd_time = (float(vx), float(vy), float(wz)), self.clock()

    def on_arm(self, servo_deg, runtime_ms):
        radians = []
        for j, (deg, c) in enumerate(zip(servo_deg, self.cal), start=1):
            q = servo_to_q(j, float(deg))
            radians.append(min(max(q, c["lower"]), c["upper"]))
        self.arm_target = (radians, max(0, min(int(runtime_ms), 5000)), self.clock())
        self.arm_dirty = True

    def on_beep(self, on):
        if self.phase == "ready":
            self.link._request(L.BEEP, bytes([1 if on else 0]))

    # --- main tick (50 Hz) -----------------------------------------------------------------
    STUCK_RESTART_S = 15.0
    RESYNC_S = 2.0
    _clock_outlier = None

    def tick(self):
        now = self.clock()
        # Self-healing: a link that never reaches "ready" (e.g. the controller keeps answering EXPIRED) is cured by a
        # fresh process; systemd restarts the bridge (Restart=on-failure).
        if self.phase == "ready":
            self._not_ready_since = None
        else:
            self._not_ready_since = getattr(self, "_not_ready_since", None) or now
            if now - self._not_ready_since > self.STUCK_RESTART_S and getattr(self, "_was_ready", False):
                self.log("link stuck in phase %s for %.0f s: restarting the bridge" % (self.phase, now - self._not_ready_since))
                os._exit(1)
        if self.phase == "ready":
            self._was_ready = True
        frames, self.frames = self.frames, []
        for kind, payload in frames:
            try:
                self._handle(kind, payload, now)
            except (ValueError, struct.error) as exc:
                self.log("bad frame %s: %s" % (kind, exc))
        fresh = self.last_status is not None and now - self.last_status < 0.5
        if not fresh:
            if self.phase != "searching":
                self.log("controller link lost")
                self._set_phase("searching")
            if now - self.last_hello > 0.4:
                self._hello(now)
            return
        if not (self.link.boot_id and self.state.get("boot") == self.link.boot_id):
            # STATUS is streaming but this boot is not identified yet: every request would carry a wrong boot id
            if now - self.last_hello > 0.4:
                self._hello(now)
            return
        if (self.state.get("mode") == 2 and self.phase not in ("clearing", "fault") and now >= self.ignore_fault_until):
            self.log("controller fault %s: clearing" % self.state.get("fault"))
            self._set_phase("fault")
        if self.phase == "fault":
            if self.state.get("mode") != 2:
                self._set_phase("searching")            # already cleared (a stale status made us wait)
                return
            self.link.session = max(self.link.session, self.state.get("highest_session", 0))
            self._send(L.CLEAR, new_session=True)
            self._set_phase("clearing")
        elif self.phase == "searching":
            self.link.session = max(self.link.session, self.state.get("highest_session", 0))
            self._send(L.OPEN, new_session=True)
            self._set_phase("opening")
        elif self.phase in ("opening", "clearing", "calibrating") and now - self.phase_since > 0.6:
            self.log("no answer in phase %s: retrying" % self.phase)
            self._set_phase("fault" if (self.phase == "clearing" and self.state.get("mode") == 2) else "searching")
        elif self.phase == "ready":
            # The offset was measured only while connecting; command leases are 240 ms, so crystal
            # drift made every command EXPIRED after ~25-40 min. Re-measure it every 2 s.
            if now - self.last_hello > self.RESYNC_S:
                self._hello(now)
            self._stream(now)

    def _hello(self, now):
        import secrets
        nonce = secrets.randbits(64)
        self.hello_sent = {n: t for n, t in self.hello_sent.items() if now - t[0] < 1.0}
        self.hello_sent[nonce] = (now, time.monotonic_ns())
        self.last_hello = now
        self.link.send(L.HELLO, struct.pack("<Q", nonce))

    def _set_phase(self, phase):
        self.phase, self.phase_since = phase, self.clock()

    def _send(self, kind, data=b"", lease_us=LEASE_US, new_session=False):
        seq = self.link._request(kind, data, lease_us=lease_us, new_session=new_session)
        self.pending[seq] = (kind, self.clock())
        return seq

    def _stream(self, now):
        if now - self.last_base_send >= 0.05:
            self.last_base_send = now
            vx, vy, wz = self.cmd if now - self.cmd_time < 0.25 else (0.0, 0.0, 0.0)
            self._send(L.BASE, struct.pack("<fffQ", vx, vy, wz, 0), lease_us=LEASE_US)
        if self.arm_target is None:
            return
        radians, runtime, _ = self.arm_target
        if not self.state.get("arm_enabled") or self.state.get("arm_cancel_pending"):
            if now - self.last_arm_enable > 0.3:
                self.last_arm_enable = now
                self._send(L.ARM_ENABLE)
            return
        if self.arm_dirty or now - self.last_arm_send > 0.12:
            self.last_arm_send = now
            self.arm_dirty = False
            self._send(L.ARM, struct.pack("<6fH", *radians, runtime))

    # --- frame handling ----------------------------------------------------------------------
    def _handle(self, kind, payload, now):
        self.stats["frames"] += 1
        if kind == L.IDENTITY:
            nonce = struct.unpack_from("<Q", payload)[0]
            sent = self.hello_sent.pop(nonce, None)
            if sent is None:
                return
            t_send, t_send_ns = sent
            mcu_us = struct.unpack_from("<Q", payload, 8)[0]
            boot = struct.unpack_from("<Q", payload, 16)[0]
            if payload[48] != 1 or struct.unpack_from("<I", payload, 24)[0] != 0x450:
                self.log("identity mismatch: not the expected controller")
                return
            mid = (t_send_ns + time.monotonic_ns()) // 2000
            measured = mcu_us - mid
            previous = self.link.mcu_offset_us
            step = measured - previous
            if boot != self.link.boot_id or not previous:
                self.link.mcu_offset_us = measured
            elif abs(step) > 5000:
                # one HELLO can be delayed by USB scheduling (seen: 11 ms); accept a big step only when confirmed
                if self._clock_outlier is not None and abs(step - self._clock_outlier) < 5000:
                    self.link.mcu_offset_us = measured
                    self._clock_outlier = None
                else:
                    self._clock_outlier = step
            else:
                self._clock_outlier = None
                self.link.mcu_offset_us = previous + step // 2
            if boot != self.link.boot_id:
                self.stats["boots"] += 1
                self.link.session = 0
                self.link.sequence = 0
                self.log("controller boot %x firmware %s" % (boot & 0xFFFFFFFF, payload[49:57].hex()))
            self.link.boot_id = boot
            self.identity = dict(boot=boot, source=payload[49:81].hex())
        elif kind == L.STATUS:
            st = decode_status(payload)
            if self.link.boot_id and st["boot"] != self.link.boot_id:
                self.log("controller restarted")
                self.last_status = None
                self.link.boot_id = 0
                self._set_phase("searching")
                return
            self.state, self.last_status = st, now
            self._odometry(st)
        elif kind == L.RESULT:
            acquired, sequence, op, result, mode, fault = struct.unpack("<QQBBBB", payload)
            entry = self.pending.pop(sequence, None)
            if result != 0:
                self.stats["results_rejected"] += 1
                if op not in (L.ARM_ENABLE, L.ARM):
                    self.log("controller rejected op %d: %s" % (op, L.RESULT_NAMES[result] if result < 18 else result))
            if op in (L.OPEN, L.CLEAR) and entry is not None:
                if result == 0 and self.phase in ("opening", "clearing"):
                    if op == L.CLEAR:
                        self.ignore_fault_until = now + 0.3   # statuses already in flight still show the old fault
                        self._set_phase("searching")        # a new session is needed after a clear
                    else:
                        self._send(L.CALIBRATION, self._calibration_payload())
                        self._set_phase("calibrating")
                # a rejected OPEN/CLEAR is retried by the phase timeout, not on every result
            elif op == L.CALIBRATION and entry is not None and self.phase == "calibrating":
                if result == 0:
                    self.log("controller ready (session %d)" % self.link.session)
                    self._set_phase("ready")
                else:
                    self._set_phase("searching")
            if len(self.pending) > 64:
                self.pending.clear()
        elif kind == L.BATTERY:
            acquired, error, raw, volts = struct.unpack("<QBHf", payload)
            if error == 0 and 0 < raw < 4096 and math.isfinite(volts) and 0 < volts < 20:
                self.publish("battery", volts)
        elif kind == L.IMU:
            self._imu(payload)
        elif kind in (L.LIDAR0, L.LIDAR1):
            index = kind - L.LIDAR0
            complete = self.scans[index].packet(payload)
            if complete is not None:
                self.publish("scan%d" % index, complete)
        elif kind == L.SERVO:
            event, joint, error, device, raw, acquired = struct.unpack_from("<QBBBHQ", payload)
            if 1 <= joint <= 6:
                c = self.cal[joint - 1]
                if error == 0 and 96 <= raw <= 4000:
                    deg = q_to_servo(joint, c["radians_per_tick"] * raw + c["radians_at_raw_zero"])
                    self.servo_deg[joint - 1] = deg
                else:
                    self.servo_deg[joint - 1] = None
                self.publish("arm_servo", list(self.servo_deg))

    def _calibration_payload(self):
        out = b""
        for j in self.cal:
            out += struct.pack("<HHHHffffHHI", 96, 4000, j["command_min"], j["command_max"], j["radians_per_tick"],
                               j["radians_at_raw_zero"], j["lower"], j["upper"], j.get("recovery_min", 0),
                               j.get("recovery_max", 0), 0)
        return out

    def _odometry(self, st):
        if not st["encoder_measurement_valid"]:
            self.last_odom_us = None
            return
        t = st["acquired_us"]
        vx, vy, wz = st["velocity"]
        if self.last_odom_us is not None:
            dt = (t - self.last_odom_us) / 1e6
            if 0 < dt < 0.2:
                yaw = self.pose[2] + wz * dt / 2
                self.pose[0] += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
                self.pose[1] += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
                self.pose[2] += wz * dt
        self.last_odom_us = t
        self.publish("odom", dict(pose=list(self.pose), velocity=[vx, vy, wz]))

    def _imu(self, payload):
        if len(payload) != 44:
            raise ValueError("bad IMU record")
        if payload[8] == 0:
            raw = struct.unpack_from(">7h", payload, 10)
            self.publish("imu", dict(accel=[v * 9.80665 / 2048 for v in raw[:3]],
                                     gyro=[math.radians(v / 16.4) for v in raw[3:6]]))
        if payload[9] == 0:
            mag = struct.unpack_from("<3h", payload, 25)
            self.publish("mag", [v * 0.15e-6 for v in mag])
