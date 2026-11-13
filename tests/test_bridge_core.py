import json
import struct
import time
from pathlib import Path

import pytest

from rosmaster import link as L
from rosmaster.core import BridgeCore, q_to_servo, servo_to_q
from rosmaster.fake_board import FakeBoard

CAL = json.loads((Path(__file__).parent / "controller-calibration.json").read_text())["joints"]


class Rig:
    def __init__(self):
        self.board = FakeBoard()
        self.link = L.Link(self.board.slave_path, baud=None)
        self.out = {}
        self.core = BridgeCore(self.link, CAL, lambda k, v: self.out.__setitem__(k, v), log=lambda *a: None)
        self.link.callbacks.append(self.core.on_frame)

    def run(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.core.tick()
            time.sleep(0.02)

    def until(self, cond, timeout=3.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.core.tick()
            if cond():
                return True
            time.sleep(0.02)
        return False

    def close(self):
        self.link.close()
        self.board.close()


@pytest.fixture
def rig():
    r = Rig()
    yield r
    r.close()


def test_servo_conversions_round_trip_and_match_calibration():
    for j in range(1, 7):
        for deg in (35.0, 90.0, 120.0):
            assert abs(q_to_servo(j, servo_to_q(j, deg)) - deg) < 1e-9
    # servo 90 deg is raw 2000 for joints 1-4 and 6 (calibration zero point)
    for j in (1, 2, 3, 4):
        c = CAL[j - 1]
        assert abs((servo_to_q(j, 90.0) - c["radians_at_raw_zero"]) / c["radians_per_tick"] - 2000) < 1


def test_connects_opens_session_and_uploads_calibration(rig):
    assert rig.until(lambda: rig.core.phase == "ready")
    assert rig.board.calibration is not None and len(rig.board.calibration) == 192
    assert rig.board.mode == 1


def test_cmd_vel_is_streamed_and_stops_when_commands_stop(rig):
    assert rig.until(lambda: rig.core.phase == "ready")
    end = time.monotonic() + 0.4
    while time.monotonic() < end:                  # /cmd_vel is a stream: republish like a joystick node would
        rig.core.on_cmd_vel(0.1, -0.05, 0.3)
        rig.run(0.04)
    moving = [c for c in rig.board.base_commands if abs(c[0] - 0.1) < 1e-6 and abs(c[1] + 0.05) < 1e-6]
    assert len(moving) >= 4
    rig.run(0.6)                                   # no new /cmd_vel: zeros keep the lease alive
    assert rig.board.base_commands[-1][:3] == (0.0, 0.0, 0.0)
    assert rig.board.mode == 1 and not any(e[0] == "expired" for e in rig.board.log)


def test_arm_command_enables_then_streams_targets_with_runtime(rig):
    assert rig.until(lambda: rig.core.phase == "ready")
    rig.core.on_arm([90, 90, 90, 90, 90, 90], 300)
    assert rig.until(lambda: len(rig.board.arm_targets) >= 2)
    q, runtime = rig.board.arm_targets[-1]
    assert runtime == 300 and abs(q[1]) < 1e-3 and abs(q[0]) < 1e-3
    assert rig.board.arm_enabled


def test_odometry_battery_imu_and_servo_feedback_are_published(rig):
    assert rig.until(lambda: rig.core.phase == "ready")
    def drive():
        rig.core.on_cmd_vel(0.2, 0.0, 0.0)
        return "odom" in rig.out and rig.out["odom"]["pose"][0] > 0.02
    assert rig.until(drive, 3.0)
    assert rig.until(lambda: "battery" in rig.out and "imu" in rig.out and "arm_servo" in rig.out, 3.0)
    assert abs(rig.out["battery"] - 11.9) < 1e-3
    assert abs(rig.out["imu"]["accel"][2] - 9.80665) < 1e-3
    assert rig.until(lambda: all(v is not None for v in rig.out["arm_servo"]), 3.0)
    assert all(abs(v - 90.0) < 1.0 for v in rig.out["arm_servo"][:4])      # raw 2000 is 90 deg for joints 1-4


def test_controller_fault_is_cleared_and_stream_resumes(rig):
    assert rig.until(lambda: rig.core.phase == "ready")
    rig.board.inject_fault()
    assert rig.until(lambda: rig.board.mode == 1 and rig.core.phase == "ready", 5.0)
    rig.core.on_cmd_vel(0.1, 0, 0)
    rig.run(0.3)
    assert any(abs(c[0] - 0.1) < 1e-6 for c in rig.board.base_commands[-6:])


def test_bridge_started_while_controller_already_streaming_and_faulted():
    board = FakeBoard()
    try:
        time.sleep(0.3)                                  # status frames are flowing before the bridge exists
        board.inject_fault()
        lk = L.Link(board.slave_path, baud=None)
        out = {}
        core = BridgeCore(lk, CAL, lambda k, v: out.__setitem__(k, v), log=lambda *a: None)
        lk.callbacks.append(core.on_frame)
        end = time.monotonic() + 5
        while time.monotonic() < end and core.phase != "ready":
            core.tick()
            time.sleep(0.02)
        assert core.phase == "ready" and board.mode == 1
        assert core.stats["results_rejected"] < 5
        lk.close()
    finally:
        board.close()
