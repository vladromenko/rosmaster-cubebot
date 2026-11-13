"""No-motion diagnostics: sensor health, odometry noise, floor plane vs arm model, detections."""
import math
import time

import cv2
import numpy as np

from .vision import detect_box, detect_cubes, fit_floor, gravity_aligned_pose, to_base


def run(io, cfg, log, image_out):
    arm = io.arm
    log("battery: %s V" % io.battery())
    io.odometry.set_stationary(True)
    t0 = time.monotonic()
    p0 = io.pose()
    time.sleep(3.0)
    p1 = io.pose()
    log("stationary 3 s: fused pose drift dx=%.4f dy=%.4f dyaw=%.4f rad; gyro bias %.5f rad/s; icp %s" % (
        p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2], io.odometry._gyro_bias, io.odometry.stats))
    pts = io.scan_points()
    log("merged scan: %s points" % (None if pts is None else len(pts)))
    frame = io._latest_frame()
    if frame is None:
        log("no camera frame")
        return
    valid = frame.depth[frame.depth > 0]
    log("depth: %.0f%% valid, min %.3f median %.3f max %.3f m" % (
        100 * valid.size / frame.depth.size, valid.min(), np.median(valid), valid.max()))
    servo = io.arm_command()
    t_fk = arm.camera_pose(servo)
    down = t_fk[:3, :3].T @ np.array([0., 0., -1.])
    floor = fit_floor(frame.depth, frame.K, down, cfg["camera"])
    log("commanded arm %s -> FK camera height %.3f m, pitch %.1f deg" % (
        servo, t_fk[2, 3], math.degrees(math.asin(-t_fk[2, 2]))))
    if floor is None:
        log("floor plane: not found (camera may not see the floor in this pose)")
        t_cam = t_fk
    else:
        tilt = math.degrees(math.acos(max(-1., min(1., float(floor.down @ down)))))
        log("floor plane: height %.3f m (FK %.3f), tilt vs FK %.1f deg, inliers %.0f%%" % (
            floor.height, t_fk[2, 3], tilt, 100 * floor.inlier_ratio))
        t_cam = gravity_aligned_pose(t_fk, floor)
    out = cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2BGR)
    for obs in detect_cubes(frame, cfg["cube"], cfg["camera"]):
        base = to_base(t_cam, obs.cam_xyz)
        log("cube: pixel (%.0f, %.0f) range %.3f m depth_valid=%s clipped=%s base %s" % (
            obs.u, obs.v, obs.range_m, obs.depth_valid, obs.clipped, np.round(base, 3)))
        cv2.circle(out, (int(obs.u), int(obs.v)), 12, (0, 0, 255), 2)
    box = detect_box(frame, t_cam, 0.0, cfg["box"], cfg["camera"])
    if box is not None:
        log("box: centre %s size %.2f x %.2f rim %.3f m near rim %s" % (
            np.round(box.center_xy, 3), box.length, box.width, box.rim_height, np.round(box.near_xy, 3)))
    cv2.imwrite(image_out, out)
    log("annotated frame saved to %s" % image_out)
