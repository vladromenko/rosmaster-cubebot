import numpy as np

from cubebot import config, mirror
from cubebot.arm_model import ArmModel


def make(**over):
    cfg = config.load()
    arm = ArmModel(cfg["arm"]["tcp_offset_m"])
    # Mapper math on the documented defaults, independent of the operator-tuned config/mirror.json.
    m = dict(mirror.DEFAULTS)
    m["relative"] = False
    m.update(over)
    return cfg, arm, mirror.MirrorMapper(arm, cfg["arm"], m, cfg["arm"]["search_pose"])


LEADER0 = dict(shoulder_pan=0, shoulder_lift=0, elbow_flex=0, wrist_flex=0, wrist_roll=0, gripper=0)


def run(mapper, leader, seconds, t0=0.0):
    cmds, dt, t = [], 0.033, t0
    for _ in range(int(seconds / dt)):
        t += dt
        mapper.update(leader, t)
        c = mapper.step(dt, t)
        if c:
            cmds.append(c)
    return cmds, t


def test_neutral_leader_maps_to_neutral_servos_with_gripper_open():
    cfg, arm, mp = make(smoothing=1.0)
    mp.update(LEADER0, 0.0)
    assert np.allclose(mp.target[:5], 90) and mp.target[5] == cfg["arm"]["gripper_open"]


def test_sign_scale_and_gripper_mapping():
    cfg, arm, mp = make(smoothing=1.0, scale=[1, 1, 1, 1, 1], sign=[1, -1, 1, 1, 1])
    mp.update(dict(LEADER0, shoulder_pan=20, shoulder_lift=10, gripper=100), 0.0)
    assert abs(mp.target[0] - 110) < 1e-9 and abs(mp.target[1] - 80) < 1e-9
    assert mp.target[5] == cfg["arm"]["gripper_close"]


def test_speed_limit_and_no_data_hold():
    cfg, arm, mp = make(smoothing=1.0, scale=[1, 1, 1, 1, 1], max_deg_s=60.0)
    cmds, t = run(mp, dict(LEADER0, shoulder_pan=40), 0.5)
    steps = [abs(b[0] - a[0]) for a, b in zip(cmds, cmds[1:])]
    assert steps and max(steps) <= 60.0 * 0.033 + 1.6
    assert mp.step(0.033, t + 2.0) is None and mp.blocked == "no leader data"     # leader silent: hold


def test_unsafe_pose_is_never_commanded_and_garbage_is_ignored():
    cfg, arm, mp = make(smoothing=1.0, scale=[1, 1, 1, 1, 1])
    cmds, _ = run(mp, dict(LEADER0, shoulder_lift=-80, elbow_flex=-90, wrist_flex=-90), 2.0)
    assert all(arm.collision_free(c, cfg["arm"]["floor_margin_m"]) for c in cmds)
    assert mp.update(dict(shoulder_pan="x"), 1.0) is False and mp.update({}, 1.0) is False
