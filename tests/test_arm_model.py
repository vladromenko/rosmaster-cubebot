import math

import numpy as np

from cubebot.arm_model import ArmModel, servo_to_q, q_to_servo


def test_servo_mapping_round_trip():
    s = np.array([60., 100., 70., 120., 90.])
    assert np.allclose(q_to_servo(servo_to_q(s)), s)


def test_matches_vendor_forward_kinematics_of_init_pose():
    # Vendor (Yahboom) CurEndPos for init joints [90, 130, 0, 5, 90]: x .1605 z .2159, pitch 45 deg.
    arm = ArmModel(tcp_offset=0.0932)
    f = arm.fk([90, 130, 0, 5, 90, 30])
    assert abs(f["tcp"][0, 3] - 0.1605) < 0.004 and abs(f["tcp"][2, 3] - 0.2159) < 0.004
    assert abs(math.degrees(arm.approach_pitch_down([90, 130, 0, 5, 90, 30])) - 45.0) < 0.1


def test_camera_looks_along_gripper_with_tcp_below_axis():
    arm = ArmModel()
    tcp_cam = arm.tcp_in_camera([90, 130, 0, 5, 90, 30])
    assert tcp_cam[2] > 0.08 and 0.03 < tcp_cam[1] < 0.07 and abs(tcp_cam[0]) < 0.01


def test_ik_reaches_floor_target():
    arm = ArmModel()
    sol, pitch = arm.grasp_ik([0.21, 0.04, 0.0175], [90, 130, 0, 5, 90, 30])
    assert sol is not None
    assert np.linalg.norm(arm.tcp_position(np.append(sol, 30)) - [0.21, 0.04, 0.0175]) < 0.006


def test_ik_rejects_unreachable_target():
    sol, _ = ArmModel().ik([0.6, 0.0, 0.0175], 70, [90, 130, 0, 5, 90, 30])
    assert sol is None


def test_collision_check_blocks_floor_penetration():
    arm = ArmModel()
    assert arm.collision_free([90, 130, 0, 5, 90, 30])
    assert not arm.collision_free([90, 0, 0, 90, 90, 30])          # fingertips below the floor
    assert not arm.collision_free([90, 0, 0, 0, 90, 30])           # folded into the chassis
