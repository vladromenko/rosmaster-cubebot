import numpy as np

from cubebot import config
from cubebot.base_control import ObstacleGuard, VelocityShaper


def test_guard_scales_speed_to_zero_before_wall():
    guard = ObstacleGuard(config.load()["base"])
    wall = np.stack([np.full(50, 0.30), np.linspace(-0.3, 0.3, 50)], axis=1)   # 13 cm beyond the bumper
    vx, vy, wz = guard.limit((0.1, 0.0, 0.0), wall)
    assert vx < 0.05
    wall = np.stack([np.full(50, 0.20), np.linspace(-0.3, 0.3, 50)], axis=1)
    assert guard.limit((0.1, 0.0, 0.0), wall)[0] == 0.0


def test_guard_allows_driving_away_from_adjacent_obstacle():
    guard = ObstacleGuard(config.load()["base"])
    wall = np.stack([np.full(50, 0.15), np.linspace(-0.3, 0.3, 50)], axis=1)
    assert guard.limit((-0.1, 0.0, 0.0), wall)[0] == -0.1


def test_shaper_limits_speed_and_acceleration():
    shaper = VelocityShaper(config.load()["base"])
    out = shaper.step((5.0, 0.0, 0.0), 0.05)
    assert 0 < out[0] <= 0.25 * 0.05 + 1e-9
    for _ in range(100):
        out = shaper.step((5.0, 0.0, 0.0), 0.05)
    assert abs(out[0] - config.load()["base"]["max_vx"]) < 1e-9
