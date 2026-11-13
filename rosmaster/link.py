"""Host side of the Rosmaster controller protocol (COBS + CRC32 frames over USART1 at 2 Mbaud).

Frames and the session/lease model follow firmware/core (explorer_wire.h, controller.c). This module owns
the serial port; the ROS bridge and the tools build on it.
"""
import struct
import threading
import time
import zlib

import serial

BOOT_KEY = 0xB007D0ADC0DEFEED
MAX_PAYLOAD = 1024
MAX_FRAME = 1040

HELLO, OPEN, BASE, HOLD, CANCEL, ESTOP, CLEAR, ARM, ARM_ENABLE, ARM_CANCEL = 1, 2, 3, 4, 5, 6, 7, 8, 9, 10
RGB, BEEP, CALIBRATION, BOOTLOADER = 11, 12, 13, 16
STATUS, RESULT, SERVO, IMU, BATTERY, LIDAR0, LIDAR1, IDENTITY, ARM_DIAG = 64, 65, 66, 67, 68, 69, 70, 71, 73
RESULT_NAMES = ("OK", "ARGUMENT", "VALUE", "RANGE", "TIME", "DISARMED", "LATCHED", "SESSION", "REPLAY", "EXPIRED",
                "FRAME", "CHECKSUM", "DEVICE_ERROR", "UNCALIBRATED", "STALE", "SOFT_LIMIT", "NOT_READY", "TICK_OVERRUN")


def encode(kind, payload=b""):
    raw = struct.pack("<BBH", 1, kind, len(payload)) + payload
    raw += struct.pack("<I", zlib.crc32(raw))
    out = bytearray([0])
    code_at, code = 0, 1
    for byte in raw:
        if byte == 0:
            out[code_at] = code
            code_at, code = len(out), 1
            out.append(0)
        else:
            out.append(byte)
            code += 1
            if code == 255:
                out[code_at] = code
                code_at, code = len(out), 1
                out.append(0)
    out[code_at] = code
    out.append(0)
    return bytes(out)


def decode(encoded):
    if not encoded or len(encoded) >= MAX_FRAME or 0 in encoded:
        raise ValueError("invalid COBS envelope")
    raw, index = bytearray(), 0
    while index < len(encoded):
        code = encoded[index]
        index += 1
        if index + code - 1 > len(encoded):
            raise ValueError("truncated COBS block")
        raw.extend(encoded[index:index + code - 1])
        index += code - 1
        if code < 255 and index < len(encoded):
            raw.append(0)
    if len(raw) < 8:
        raise ValueError("short frame")
    version, kind, length = struct.unpack_from("<BBH", raw)
    if version != 1 or length != len(raw) - 8:
        raise ValueError("bad version/length")
    if struct.unpack_from("<I", raw, len(raw) - 4)[0] != zlib.crc32(raw[:-4]):
        raise ValueError("CRC mismatch")
    return kind, bytes(raw[4:-4])


class Parser:
    def __init__(self):
        self.buffer, self.errors = bytearray(), 0

    def feed(self, data):
        frames = []
        for byte in data:
            if byte:
                if len(self.buffer) < MAX_FRAME:
                    self.buffer.append(byte)
            else:
                if self.buffer:
                    try:
                        frames.append(decode(bytes(self.buffer)))
                    except ValueError:
                        self.errors += 1
                self.buffer.clear()
        return frames


class Link:
    """Open port + reader thread. `frames` collects decoded telemetry; `latest[kind]` keeps the newest payload."""

    def __init__(self, port="/dev/ttyUSB0", baud=2_000_000):
        self.port = serial.Serial(port, baud, timeout=0.02) if baud else serial.Serial(port, timeout=0.02)
        self.parser = Parser()
        self.latest, self.counts = {}, {}
        self.callbacks = []
        self._run = True
        self._lock = threading.Lock()
        self.boot_id = 0
        self.mcu_offset_us = 0
        self.session = self.sequence = 0
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()

    def close(self):
        self._run = False
        self.thread.join(timeout=1)
        self.port.close()

    def _read(self):
        while self._run:
            data = self.port.read(4096)
            if not data:
                continue
            for kind, payload in self.parser.feed(data):
                with self._lock:
                    self.latest[kind] = (time.monotonic(), payload)
                    self.counts[kind] = self.counts.get(kind, 0) + 1
                for fn in self.callbacks:
                    fn(kind, payload)

    def send(self, kind, payload=b""):
        self.port.write(encode(kind, payload))

    # --- session -----------------------------------------------------------------
    def hello(self, timeout=2.0):
        nonce = struct.pack("<Q", time.monotonic_ns() & 0xFFFFFFFFFFFFFFFF)
        t_send = time.monotonic_ns()
        self.send(HELLO, nonce)
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            got = self.latest.get(IDENTITY)
            if got and got[1][:8] == nonce:
                p = got[1]
                t_recv = time.monotonic_ns()
                mcu_us = struct.unpack_from("<Q", p, 8)[0]
                self.boot_id = struct.unpack_from("<Q", p, 16)[0]
                self.mcu_offset_us = mcu_us - (t_send + t_recv) // 2000
                return dict(boot_id=self.boot_id, devid=struct.unpack_from("<I", p, 24)[0],
                            revid=struct.unpack_from("<I", p, 28)[0], flash_kb=struct.unpack_from("<H", p, 32)[0],
                            uid=struct.unpack_from("<III", p, 36), reset_status=struct.unpack_from("<I", p, 84)[0],
                            source_digest=p[49:81].hex())
            time.sleep(0.01)
        return None

    def mcu_now_us(self):
        return time.monotonic_ns() // 1000 + self.mcu_offset_us

    def _request(self, kind, data=b"", lease_us=200_000, new_session=False):
        now = self.mcu_now_us()
        if new_session:
            self.session, self.sequence = self.session + 1, 1
        else:
            self.sequence += 1
        payload = struct.pack("<QQQQQ", self.boot_id, self.session, self.sequence, now - 2000, now + lease_us) + data
        self.send(kind, payload)
        return self.sequence

    def open_session(self, highest_seen=0):
        self.session = max(self.session, highest_seen)
        return self._request(OPEN, new_session=True)

    def base(self, vx, vy, wz, finite_us=0):
        return self._request(BASE, struct.pack("<fffQ", vx, vy, wz, finite_us), lease_us=250_000)

    def hold(self):
        return self._request(HOLD)

    def estop(self):
        self.send(ESTOP)

    def arm_enable(self, lease_us=250_000):
        return self._request(ARM_ENABLE, lease_us=lease_us)

    def arm_cancel(self):
        return self._request(ARM_CANCEL)

    def arm(self, radians, runtime_ms=0, lease_us=250_000):
        return self._request(ARM, struct.pack("<6fH", *radians, runtime_ms), lease_us=lease_us)

    def calibration(self, joints):
        """`joints`: list of six dicts from controller-calibration.json."""
        data = b""
        for j in joints:
            data += struct.pack("<HHHHffffHHI", 96, 4000, j["command_min"], j["command_max"], j["radians_per_tick"],
                                j["radians_at_raw_zero"], j["lower"], j["upper"], j.get("recovery_min", 0),
                                j.get("recovery_max", 0), 0)
        return self._request(CALIBRATION, data)

    def enter_bootloader(self):
        self.send(BOOTLOADER, struct.pack("<Q", BOOT_KEY))

    def request_bootloader(self, timeout=3.0):
        """Send the loader command until the firmware stops streaming, then fall silent at once.

        Anything sent after the jump reaches the ROM loader at the wrong baud rate and breaks its
        auto-baud detection, so the stream must end the moment the status frames stop.
        """
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            last = self.latest.get(STATUS, (0.0, b""))[0]
            if last and time.monotonic() - last > 0.25:
                return True                      # the firmware is gone: stop talking immediately
            self.enter_bootloader()
            time.sleep(0.03)
        return False

    # --- telemetry decoding ----------------------------------------------------------
    @staticmethod
    def servo(payload):
        t, joint, result, device, raw = struct.unpack_from("<QBBBH", payload)
        signed = raw - 65536 if raw >= 32768 else raw
        return dict(joint=joint, result=RESULT_NAMES[result] if result < len(RESULT_NAMES) else result,
                    device_error=device, raw=raw, signed=signed, mcu_us=t)

    @staticmethod
    def battery(payload):
        t, status, raw = struct.unpack_from("<QBH", payload)
        volts = struct.unpack_from("<f", payload, 11)[0] if status == 0 else None
        return dict(status=status, raw=raw, volts=volts)
