"""Small SE(2)/SE(3) helpers shared by every CubeBot module."""
import math

import numpy as np


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def transform(xyz=(0., 0., 0.), rotation=None):
    m = np.eye(4)
    if rotation is not None:
        m[:3, :3] = rotation
    m[:3, 3] = xyz
    return m


def axis_angle(axis, angle):
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k


def rotation_between(a, b):
    """Smallest rotation that maps unit vector a onto unit vector b."""
    a = np.asarray(a, float) / np.linalg.norm(a)
    b = np.asarray(b, float) / np.linalg.norm(b)
    v = np.cross(a, b)
    c = float(a @ b)
    s = np.linalg.norm(v)
    if s < 1e-12:
        if c > 0:
            return np.eye(3)
        perp = np.cross(a, [1., 0., 0.])
        if np.linalg.norm(perp) < 1e-6:
            perp = np.cross(a, [0., 1., 0.])
        return axis_angle(perp, math.pi)
    return axis_angle(v / s, math.atan2(s, c))


def invert(t):
    out = np.eye(4)
    out[:3, :3] = t[:3, :3].T
    out[:3, 3] = -t[:3, :3].T @ t[:3, 3]
    return out


# SE(2) poses are (x, y, yaw) tuples.
def se2_compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def se2_inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-(c * a[0] + s * a[1]), -(-s * a[0] + c * a[1]), wrap(-a[2]))


def se2_between(a, b):
    """Pose of b expressed in the frame of a."""
    return se2_compose(se2_inverse(a), b)


def se2_apply(pose, points):
    points = np.asarray(points, float)
    return points @ rot_z(pose[2]).T + np.array(pose[:2])
