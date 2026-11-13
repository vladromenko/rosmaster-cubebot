import numpy as np

from cubebot import config
from cubebot.arm_model import ArmModel
from cubebot.sim import SimRobot, World
from cubebot.vision import detect_box, detect_cubes, fit_floor, gravity_aligned_pose, to_base


def scene():
    cfg = config.load()
    arm = ArmModel()
    sim = SimRobot(cfg, arm, World(cube=(0.45, 0.05), box=(0.8, -0.35, 0.0)), seed=1, log=lambda *_: None)
    sim.move_arm(cfg["arm"]["search_pose"])
    sim.wait(0.5)
    return cfg, arm, sim, sim.render()


def test_cube_and_floor_and_box():
    cfg, arm, sim, frame = scene()
    t_fk = arm.camera_pose(sim.arm_command())
    floor = fit_floor(frame.depth, frame.K, t_fk[:3, :3].T @ [0, 0, -1.0], cfg["camera"])
    assert floor is not None and abs(floor.height - t_fk[2, 3]) < 0.03
    t = gravity_aligned_pose(t_fk, floor)
    cubes = detect_cubes(frame, cfg["cube"], cfg["camera"])
    assert len(cubes) == 1
    assert np.linalg.norm(to_base(t, cubes[0].cam_xyz)[:2] - sim.world.cube[:2]) < 0.03
    box = detect_box(frame, t, 0.0, cfg["box"], cfg["camera"])
    assert box is not None and np.linalg.norm(box.center_xy - sim.world.box_center) < 0.03
    assert abs(box.rim_height - 0.10) < 0.02


def test_no_false_cube_on_plain_floor():
    cfg, arm, sim, _ = scene()
    sim.world.cube[:] = (3.0, 3.0, 0.0175)
    assert detect_cubes(sim.render(), cfg["cube"], cfg["camera"]) == []
