import math

import numpy as np

from cubebot.geometry import se2_apply
from cubebot.scan_matching import build_scan, icp


def room_scan(pose, rng=None):
    """Points of a 4x3 m room with a corner obstacle, seen from `pose`, in the sensor frame."""
    angles = np.linspace(-math.pi, math.pi, 500, endpoint=False)
    walls = [(0, 2.0), (0, -2.0), (1, 1.5), (1, -1.5)]
    origin = np.array(pose[:2])
    pts = []
    for a in angles:
        d = np.array([math.cos(a + pose[2]), math.sin(a + pose[2])])
        best = 1e9
        for axis, bound in walls:
            if abs(d[axis]) > 1e-9:
                t = (bound - origin[axis]) / d[axis]
                if t > 0:
                    best = min(best, t)
        pts.append(best * np.array([math.cos(a), math.sin(a)]))
    pts = np.array(pts)
    if rng is not None:
        pts += rng.normal(0, 0.003, pts.shape)
    return pts, np.ones(len(pts), bool)


def test_icp_recovers_motion():
    rng = np.random.default_rng(3)
    a = build_scan([room_scan((0.3, 0.2, 0.1), rng)])
    true = (0.04, -0.02, 0.05)                           # b is a pose in a's frame
    b_pose = (0.3 + true[0] * math.cos(0.1) - true[1] * math.sin(0.1),
              0.2 + true[0] * math.sin(0.1) + true[1] * math.cos(0.1), 0.1 + true[2])
    b = build_scan([room_scan(b_pose, rng)])
    res = icp(a, b, initial=(0.0, 0.0, 0.0))
    assert res is not None and not res.degenerate
    assert np.allclose(res.delta, true, atol=(0.008, 0.008, 0.01))


def test_icp_flags_featureless_corridor_as_degenerate():
    xs = np.linspace(-3, 3, 300)
    wall = np.concatenate([np.stack([xs, np.full_like(xs, 0.5)], 1), np.stack([xs, np.full_like(xs, -0.5)], 1)])
    scan = build_scan([(wall, np.ones(len(wall), bool))])
    res = icp(scan, scan)
    assert res is not None and res.degenerate
