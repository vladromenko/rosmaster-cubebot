"""Software model of the controller (native protocol) for testing the bridge without hardware."""
import os
import select
import struct
import threading
import time

from . import link as L


class FakeBoard:
    def __init__(self):
        self.master, slave = os.openpty()
        self.slave_path = os.ttyname(slave)
        self._slave = slave
        self.parser = L.Parser()
        self.boot = 0x1122334455667788
        self.t0 = time.monotonic()
        self.session = self.highest = 0
        self.mode = 0           # 0 disarmed, 1 active, 2 fault
        self.fault = 0
        self.arm_enabled = False
        self.sequence = 0
        self.velocity = [0.0, 0.0, 0.0]
        self.cmd_expires = 0
        self.calibration = None
        self.arm_targets = []
        self.base_commands = []
        self.received = []
        self.motor_pose = [0.0, 0.0, 0.0]
        self.log = []
        self._run = True
        self._lock = threading.Lock()
        threading.Thread(target=self._read, daemon=True).start()
        threading.Thread(target=self._emit, daemon=True).start()

    def now_us(self):
        return int((time.monotonic() - self.t0) * 1e6)

    def _send(self, kind, payload):
        try:
            os.write(self.master, L.encode(kind, payload))
        except OSError:
            pass

    def inject_fault(self):
        with self._lock:
            self.mode, self.fault = 2, 5

    def _read(self):
        while self._run:
            try:
                if not select.select([self.master], [], [], 0.1)[0]:
                    continue
                data = os.read(self.master, 4096)
            except (OSError, ValueError):
                break
            for kind, payload in self.parser.feed(data):
                with self._lock:
                    self._handle(kind, payload)

    def _result(self, seq, op, result):
        self._send(L.RESULT, struct.pack("<QQBBBB", self.now_us(), seq, op, result, self.mode, self.fault))

    def _handle(self, kind, p):
        self.received.append((kind, p))
        if kind == L.HELLO:
            out = bytearray(88)
            out[0:8] = p[:8]
            struct.pack_into("<QQ", out, 8, self.now_us(), self.boot)
            struct.pack_into("<IIH", out, 24, 0x450, 0x2003, 1024)
            out[48] = 1
            out[49:81] = bytes(range(32))
            self._send(L.IDENTITY, bytes(out))
            return
        if kind == L.ESTOP:
            self.mode, self.fault, self.arm_enabled = 2, 6, False
            return
        if len(p) < 40:
            return
        boot, session, seq, issued, expiry = struct.unpack_from("<QQQQQ", p)
        now = self.now_us()
        if boot != self.boot:
            return self._result(seq, kind, 7)
        if expiry <= now or expiry - issued > 250_000:
            self.log.append(("expired", kind, expiry - now))
            return self._result(seq, kind, 9)
        if kind in (L.OPEN, L.CLEAR):
            if session <= self.highest:
                return self._result(seq, kind, 7)
            if kind == L.CLEAR:
                self.mode = 0
                self.fault = 0
            else:
                if self.mode == 2:
                    return self._result(seq, kind, 6)
                self.mode = 1
            self.session = self.highest = session
            return self._result(seq, kind, 0)
        if session != self.session:
            return self._result(seq, kind, 7)
        if self.mode != 1:
            return self._result(seq, kind, 5 if self.mode == 0 else 6)
        if kind == L.BASE:
            vx, vy, wz, finite = struct.unpack_from("<fffQ", p, 40)
            self.base_commands.append((vx, vy, wz, time.monotonic()))
            self.velocity, self.cmd_expires = [vx, vy, wz], expiry
        elif kind == L.CALIBRATION:
            if len(p) != 232:
                return self._result(seq, kind, 10)
            self.calibration = p[40:]
        elif kind == L.ARM_ENABLE:
            self.arm_enabled = True
        elif kind == L.ARM:
            self.arm_targets.append((struct.unpack_from("<6f", p, 40), struct.unpack_from("<H", p, 64)[0]))
        self.sequence = seq
        self._result(seq, kind, 0)

    def _emit(self):
        n = 0
        while self._run:
            time.sleep(0.02)
            n += 1
            with self._lock:
                now = self.now_us()
                if now >= self.cmd_expires:
                    self.velocity = [0.0, 0.0, 0.0]
                p = bytearray(216)
                struct.pack_into("<6Q", p, 0, now, self.boot, self.session, self.sequence, self.cmd_expires, 0)
                p[48], p[49], p[50], p[51] = self.mode, self.fault, int(self.arm_enabled), 0
                for i in range(4):
                    struct.pack_into("<HI", p, 52 + 26 * i, 0, 0)
                    struct.pack_into("<I", p, 52 + 26 * i + 22, (1, 3, 2, 4)[i])
                struct.pack_into("<3f", p, 156, *self.velocity)
                struct.pack_into("<Q", p, 196, self.highest)
                struct.pack_into("<I", p, 212, 1)
                self._send(L.STATUS, bytes(p))
                if n % 25 == 0:
                    self._send(L.BATTERY, struct.pack("<QBHf", now, 0, 2900, 11.9))
                    imu = bytearray(44)
                    struct.pack_into(">7h", imu, 10, 0, 0, 2048, 164, 0, 0, 0)
                    imu[9] = 1
                    self._send(L.IMU, bytes(imu))
                    for joint in range(1, 7):
                        self._send(L.SERVO, struct.pack("<QBBBHQ8sB", now, joint, 0, 0, 2000, now, b"\xff\xf5" + bytes(6), 0xf5))

    def close(self):
        self._run = False
        time.sleep(0.25)
        for fd in (self.master, self._slave):
            try:
                os.close(fd)
            except OSError:
                pass
