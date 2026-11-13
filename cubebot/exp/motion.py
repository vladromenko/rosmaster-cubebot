"""Closed-loop body motions on odometry (shared by gestures, retrieval and the assistant)."""
import math

import numpy as np

from ..geometry import wrap


def turn_by(io, angle, wz=0.5, tolerance=0.06, timeout=15.0, stop_fn=None):
    """Rotate in place by `angle` rad (positive = left). Returns the angle actually turned."""
    start = io.pose()[2]
    target = wrap(start + angle)

    def cmd():
        err = wrap(target - io.pose()[2])
        speed = max(0.15, min(wz, 1.5 * abs(err)))
        return 0.0, 0.0, math.copysign(speed, err)

    io.drive_until(cmd, lambda: abs(wrap(target - io.pose()[2])) < tolerance or (stop_fn is not None and stop_fn()),
                   timeout, "turn")
    io.wait(0.15)
    return wrap(io.pose()[2] - start)


def translate(io, dx, dy=0.0, speed=0.15, timeout=12.0, stop_fn=None):
    """Move by (dx, dy) in the *initial* body frame. Returns (travelled_x, travelled_y) in that frame."""
    p0 = io.pose()
    c, s = math.cos(p0[2]), math.sin(p0[2])
    dist = math.hypot(dx, dy)
    if dist < 1e-6:
        return 0.0, 0.0
    ux, uy = dx / dist, dy / dist

    def travelled():
        p = io.pose()
        wx, wy = p[0] - p0[0], p[1] - p0[1]
        return c * wx + s * wy, -s * wx + c * wy

    def cmd():
        # heading changes are small; command in the current body frame so a yaw drift does not bend the path
        yaw = io.pose()[2] - p0[2]
        cy, sy = math.cos(yaw), math.sin(yaw)
        vx, vy = ux * speed, uy * speed
        return cy * vx + sy * vy, -sy * vx + cy * vy, 0.0

    def done():
        tx, ty = travelled()
        return tx * ux + ty * uy >= dist - 0.01 or (stop_fn is not None and stop_fn())

    io.drive_until(cmd, done, timeout, "translate")
    io.wait(0.15)
    return travelled()


def body_delta(p0, p1):
    """Pose change p0 -> p1 expressed in the body frame of p0: (dx, dy, dyaw)."""
    c, s = math.cos(p0[2]), math.sin(p0[2])
    wx, wy = p1[0] - p0[0], p1[1] - p0[1]
    return c * wx + s * wy, -s * wx + c * wy, wrap(p1[2] - p0[2])
