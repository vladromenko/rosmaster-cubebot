import math

import numpy as np

from cubebot import config
from cubebot.arm_model import ArmModel
from cubebot.sim import SimRobot, World


def make(seed=2, **kw):
    cfg = config.load()
    return cfg, SimRobot(cfg, ArmModel(), World(**kw), seed=seed, log=lambda *_: None)


def drive(sim, vx, vy, wz, seconds):
    end = sim.now() + seconds
    while sim.now() < end:
        sim.drive(vx, vy, wz)
        sim.wait(sim.TICK)
    sim.stop_base()
    sim.wait(0.6)


def error(sim):
    est, true = sim.pose(), sim.true_pose
    return math.hypot(est[0] - true[0], est[1] - true[1]), abs(math.remainder(est[2] - true[2], 2 * math.pi))


def test_fused_odometry_beats_wheels_under_slip():
    cfg, sim = make()
    cfg["odometry"]["lidar_correction"] = True  # Exercise ICP independently of the live default.
    sim.wait(1.5)                       # learn gyro bias
    drive(sim, 0.08, 0.0, 0.0, 6.0)
    drive(sim, 0.0, 0.07, 0.0, 4.0)
    drive(sim, 0.0, 0.0, 0.4, 5.0)
    drive(sim, -0.06, 0.04, 0.0, 4.0)
    pos, yaw = error(sim)
    wheel = sim._wheel_pose
    wheel_pos = math.hypot(wheel[0] - sim.true_pose[0], wheel[1] - sim.true_pose[1])
    assert pos < 0.03 and yaw < 0.03
    assert pos < 0.25 * wheel_pos
    assert sim.odometry.stats["icp_used"] > 50


def test_without_lidar_correction_gyro_still_bounds_yaw():
    cfg = config.load()
    cfg["odometry"]["lidar_correction"] = False
    sim = SimRobot(cfg, ArmModel(), World(), seed=2, log=lambda *_: None)
    sim.wait(1.5)
    drive(sim, 0.0, 0.0, 0.4, 8.0)
    _, yaw = error(sim)
    assert yaw < 0.06
