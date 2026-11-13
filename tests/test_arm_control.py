import numpy as np

from cubebot import config
from cubebot.arm_control import CartesianJog
from cubebot.arm_model import ArmModel


def make():
    cfg = config.load()
    arm = ArmModel(cfg["arm"]["tcp_offset_m"])
    return arm, cfg, CartesianJog(arm, cfg, cfg["arm"]["search_pose"])


def run(jog, seconds, **kw):
    cmds, dt = [], 0.05
    for _ in range(int(seconds / dt)):
        c = jog.step(dt, **kw)
        if c:
            cmds.append(c)
    return cmds


def test_forward_jog_moves_tcp_smoothly_and_forward():
    arm, cfg, jog = make()
    x0 = arm.tcp_position(jog.servo)[0]
    cmds = run(jog, 1.5, move=(1, 0, 0))
    assert arm.tcp_position(jog.servo)[0] > x0 + 0.05
    steps = [np.abs(np.array(b[:4]) - np.array(a[:4])).max() for a, b in zip(cmds, cmds[1:])]
    assert max(steps) <= 4                          # no jumps: the servos get small increments


def test_speed_is_limited_and_ramps_up():
    arm, cfg, jog = make()
    jog.step(0.05, move=(1, 0, 0))
    assert abs(jog.velocity[0]) <= jog.accel * 0.05 + 1e-9
    run(jog, 3, move=(1, 0, 0))
    assert abs(jog.velocity[0]) <= jog.max_speed[0] + 1e-9


def test_never_commands_pose_through_floor():
    arm, cfg, jog = make()
    cmds = run(jog, 8, move=(0, 0, -1))
    assert all(arm.collision_free(c, cfg["arm"]["floor_margin_m"]) for c in cmds)
    assert arm.tcp_position(jog.servo)[2] >= 0.02


def test_gripper_and_workspace_limits():
    arm, cfg, jog = make()
    run(jog, 4, move=(0, 0, 0), grip_rate=1)
    assert jog.grip <= cfg["arm"]["gripper_close"]
    run(jog, 10, move=(1, 0, 0))
    assert arm.tcp_position(jog.servo)[0] <= 0.37


def test_start_pose_outside_workspace_is_not_blocked_and_idle_sends_nothing():
    cfg = config.load()
    arm = ArmModel(cfg["arm"]["tcp_offset_m"])
    jog = CartesianJog(arm, cfg, cfg["arm"]["safe_pose"])          # upright: TCP above the nominal box
    assert jog.step(0.05, (0, 0, 0)) is None and jog.blocked is None
    cmds = run(jog, 2.0, move=(0, 0, 0), pitch_rate=1)           # tilt the wrist forward from vertical
    assert cmds and jog.pitch > -89
