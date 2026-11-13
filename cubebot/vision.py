"""RGB-D perception: floor plane, yellow cube, box, gravity-aligned camera pose."""
import math
from dataclasses import dataclass, field

import cv2
import numpy as np

from .geometry import rotation_between


@dataclass
class Frame:
    rgb: np.ndarray            # HxWx3 uint8, RGB order
    depth: np.ndarray          # HxW float32 metres, 0 where invalid
    K: np.ndarray              # 3x3 intrinsics
    stamp: float = 0.0


@dataclass
class Floor:
    down: np.ndarray           # unit normal in the camera frame pointing to the floor
    height: float              # camera height above the floor (m)
    inlier_ratio: float


@dataclass
class CubeObs:
    u: float
    v: float
    width_px: float
    range_m: float
    cam_xyz: np.ndarray
    depth_valid: bool
    score: float
    clipped: bool = False              # blob touches the image border: centroid/size are unreliable
    pts_cam: np.ndarray = None         # sampled depth points of the blob (camera frame), None without depth
    base_xyz: np.ndarray = None


@dataclass
class BoxObs:
    center_xy: np.ndarray
    yaw: float
    length: float
    width: float
    rim_height: float
    points: int = 0
    near_xy: np.ndarray = None         # rim point nearest the robot along the line to the centre


def depth_from_uint16(raw, scale=0.001):
    return raw.astype(np.float32) * scale


def deproject(u, v, z, K):
    return np.array([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z])


def cloud(depth, K, stride=4, min_z=0.15, max_z=3.0):
    h, w = depth.shape
    vs, us = np.mgrid[0:h:stride, 0:w:stride]
    z = depth[::stride, ::stride]
    ok = (z > min_z) & (z < max_z)
    z = z[ok]
    return np.stack([(us[ok] - K[0, 2]) * z / K[0, 0], (vs[ok] - K[1, 2]) * z / K[1, 1], z], axis=1)


def fit_floor(depth, K, expected_down, cfg, rng=None, max_tilt_deg=22.0):
    """RANSAC the floor plane near the FK-predicted orientation."""
    rng = rng or np.random.default_rng(0)
    pts = cloud(depth, K, stride=6, min_z=cfg["min_depth_m"], max_z=cfg["max_depth_m"])
    if len(pts) < 200:
        return None
    best, best_count = None, 0
    cos_lim = math.cos(math.radians(max_tilt_deg))
    expected_down = np.asarray(expected_down, float) / np.linalg.norm(expected_down)
    for _ in range(120):
        a, b, c = pts[rng.choice(len(pts), 3, replace=False)]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        if n @ expected_down < 0:
            n = -n
        if n @ expected_down < cos_lim:
            continue
        height = float(n @ a)
        if not 0.05 < height < 0.9:
            continue
        count = int(np.sum(np.abs(pts @ n - height) < cfg["floor_inlier_m"]))
        if count > best_count:
            best, best_count = (n, height), count
    if best is None or best_count < 0.06 * len(pts):
        return None
    n, height = best
    for _ in range(3):  # least-squares refinement on the inliers
        inl = np.abs(pts @ n - height) < cfg["floor_inlier_m"]
        if inl.sum() < 50:
            break
        centroid = pts[inl].mean(0)
        _, _, vt = np.linalg.svd(pts[inl] - centroid)
        n = vt[2]
        if n @ expected_down < 0:
            n = -n
        height = float(n @ centroid)
    return Floor(n / np.linalg.norm(n), height, best_count / len(pts))


def gravity_aligned_pose(t_fk, floor):
    """Replace the camera roll/pitch/height of the FK pose with the floor-plane measurement.

    The servos are not measured, so FK may be a few degrees off; the floor is a
    free gravity sensor. Heading (yaw about the vertical) still comes from FK.
    """
    if floor is None:
        return t_fk
    r_fk = t_fk[:3, :3]
    up_pred = r_fk.T @ np.array([0., 0., 1.])
    up_meas = -floor.down
    rc = rotation_between(up_pred, up_meas)
    out = np.eye(4)
    out[:3, :3] = r_fk @ rc.T
    out[:3, 3] = [t_fk[0, 3], t_fk[1, 3], floor.height]
    return out


def to_base(t_base_cam, cam_xyz):
    return (t_base_cam @ np.append(cam_xyz, 1.0))[:3]


def _hsv_mask(rgb, lo, hi):
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)


def detect_cubes(frame, cube_cfg, cam_cfg):
    mask = _hsv_mask(frame.rgb, cube_cfg["hsv_lo"], cube_cfg["hsv_hi"])
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    K, size = frame.K, cube_cfg["size_m"]
    height, width = mask.shape
    out = []
    for i in range(1, count):
        x, y, w, h, area = stats[i]
        if area < cube_cfg["min_pixels"]:
            continue
        clipped = x <= 1 or y <= 1 or x + w >= width - 1 or y + h >= height - 1
        fill = area / float(w * h)
        aspect = w / float(h)
        if not clipped and (fill < 0.55 or not 0.45 < aspect < 2.2):
            continue
        if clipped and fill < 0.5:
            continue
        width_px = math.sqrt(area) if not clipped else float(max(w, h) if (x <= 1 or x + w >= width - 1) else w)
        blob = labels[y:y + h, x:x + w] == i
        core = cv2.erode(blob.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
        if core.sum() < 5:
            core = blob
        d = frame.depth[y:y + h, x:x + w][core]
        d = d[(d > cam_cfg["min_depth_m"]) & (d < cam_cfg["max_depth_m"])]
        u, v = centroids[i]
        range_size = K[0, 0] * size / width_px
        tol = cube_cfg["size_tolerance"]
        if len(d) >= max(4, 0.25 * core.sum()):
            z = float(np.median(d))
            real = width_px * z / K[0, 0]
            if not clipped and not size * (1 - tol) < real < size * (1 + tol) * 1.6:
                continue
            valid = True
        else:
            # Too close for the depth sensor: range from the apparent size.
            z = range_size
            valid = False
            if not 0.06 < z < 0.8:
                continue
        pts_cam = None
        if valid and not clipped:
            ys, xs = np.nonzero(core & (frame.depth[y:y + h, x:x + w] > cam_cfg["min_depth_m"]))
            if len(ys) > 8:
                pick = np.linspace(0, len(ys) - 1, min(len(ys), 150)).astype(int)
                zz = frame.depth[y:y + h, x:x + w][ys[pick], xs[pick]]
                pts_cam = np.stack([(xs[pick] + x - K[0, 2]) * zz / K[0, 0], (ys[pick] + y - K[1, 2]) * zz / K[1, 1], zz], 1)
        z += 0.45 * size               # depth and apparent size describe the near surface, not the centre
        score = fill * min(1.0, area / 400.0) * (0.6 if clipped else 1.0)
        out.append(CubeObs(float(u), float(v), width_px, z, deproject(u, v, z, K), valid, score, clipped, pts_cam))
    out.sort(key=lambda o: -o.score)
    return out


def cube_in_hand(rgb, depth, roi, cube_cfg, min_fraction=0.18, near_m=0.20):
    """Yellow share of the gripper ROI (rows r0:r1, cols c0:c1) at the bottom of the image.

    The ROI also looks at the floor further ahead, so only yellow pixels whose depth is
    missing (closer than the sensor can measure) or below `near_m` count as in the hand.
    """
    r0, r1, c0, c1 = roi
    mask = _hsv_mask(rgb[r0:r1, c0:c1], cube_cfg["hsv_lo"], cube_cfg["hsv_hi"]) > 0
    d = depth[r0:r1, c0:c1]
    near = mask & ((d <= 0.0) | (d < near_m))
    return float(np.count_nonzero(near)) / mask.size >= min_fraction


def detect_box(frame, t_base_cam, floor_height_base, box_cfg, cam_cfg):
    """Find an open box/container; returns BoxObs in the base frame or None."""
    K = frame.K
    if box_cfg["mode"] == "hsv":
        mask = (_hsv_mask(frame.rgb, box_cfg["hsv_lo"], box_cfg["hsv_hi"]) > 0)[::4, ::4]
        depth = frame.depth[::4, ::4]
        vs, us = np.mgrid[0:frame.depth.shape[0]:4, 0:frame.depth.shape[1]:4]
        ok = mask & (depth > cam_cfg["min_depth_m"]) & (depth < cam_cfg["max_depth_m"])
        z = depth[ok]
        pts_cam = np.stack([(us[ok] - K[0, 2]) * z / K[0, 0], (vs[ok] - K[1, 2]) * z / K[1, 1], z], axis=1)
    else:
        pts_cam = cloud(frame.depth, K, stride=3, min_z=cam_cfg["min_depth_m"], max_z=cam_cfg["max_depth_m"])
    if len(pts_cam) < 30:
        return None
    pts = (t_base_cam[:3, :3] @ pts_cam.T).T + t_base_cam[:3, 3]
    pts[:, 2] -= floor_height_base
    pts = pts[(pts[:, 2] > box_cfg["min_height_m"] * 0.5) & (pts[:, 2] < box_cfg["max_height_m"] * 1.8)]
    if len(pts) < 30:
        return None
    # Top-down occupancy grid, 2 cm cells; connected components are candidate objects.
    cell = 0.02
    xy0 = pts[:, :2].min(0) - 0.05
    ij = np.floor((pts[:, :2] - xy0) / cell).astype(int)
    grid = np.zeros((ij[:, 0].max() + 3, ij[:, 1].max() + 3), np.uint8)
    grid[ij[:, 0], ij[:, 1]] = 255
    grid = cv2.morphologyEx(grid, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    count, labels = cv2.connectedComponents(grid)
    best, best_score = None, 0.0
    for lab in range(1, count):
        member = labels[ij[:, 0], ij[:, 1]] == lab
        if member.sum() < 25:
            continue
        cl = pts[member]
        zmax = float(np.percentile(cl[:, 2], 95))
        if zmax > box_cfg["max_height_m"] or zmax < box_cfg["min_height_m"]:
            continue
        # The rim band outlines the box even when only some walls face the camera.
        band = cl[cl[:, 2] > zmax - 0.04][:, :2]
        if len(band) < 15:
            continue
        (cx, cy), (rw, rh), ang = cv2.minAreaRect(band.astype(np.float32))
        length, width = max(rw, rh), min(rw, rh)
        if not (box_cfg["min_width_m"] <= length <= box_cfg["max_width_m"] and width >= 0.08):
            continue
        yaw = math.radians(ang if rw >= rh else ang + 90.0)
        center = np.array([cx, cy])
        u = center / max(np.linalg.norm(center), 1e-6)
        near = band[np.argsort(band @ u)[:max(1, len(band) // 30)]].mean(0)
        score = len(cl)
        if score > best_score:
            best, best_score = BoxObs(center, yaw, length, width, zmax, len(cl), near), score
    return best


def _project_to_pixel(point_base, t_base_cam, K):
    cam = np.linalg.inv(t_base_cam) @ np.append(point_base, 1.0)
    if cam[2] <= 0.05:
        return None
    return K[0, 0] * cam[0] / cam[2] + K[0, 2], K[1, 1] * cam[1] / cam[2] + K[1, 2]


def learn_box_color(frame, t_base_cam, box, half=14):
    """Colour model of the box, sampled where the depth detector found it (so it follows whatever box is there).

    Returns {"lo": [h, s, v], "hi": [h, s, v], "rim": rim_height, "length": l, "width": w} or None.
    """
    uv = _project_to_pixel(np.array([box.center_xy[0], box.center_xy[1], box.rim_height * 0.8]), t_base_cam, frame.K)
    if uv is None:
        return None
    u, v = int(round(uv[0])), int(round(uv[1]))
    h, w = frame.rgb.shape[:2]
    r0, r1, c0, c1 = max(0, v - half), min(h, v + half), max(0, u - half), min(w, u + half)
    if r1 - r0 < 8 or c1 - c0 < 8:
        return None
    hsv = cv2.cvtColor(frame.rgb[r0:r1, c0:c1], cv2.COLOR_RGB2HSV).reshape(-1, 3).astype(np.float32)
    med = np.median(hsv, axis=0)
    spread = np.array([10.0, 45.0, 60.0])
    lo = np.clip(med - spread, [0, 0, 0], [179, 255, 255])
    hi = np.clip(med + spread, [0, 0, 0], [179, 255, 255])
    return dict(lo=lo.tolist(), hi=hi.tolist(), rim=float(box.rim_height), length=float(box.length), width=float(box.width))


def detect_box_by_color(frame, t_base_cam, model, min_pixels=500):
    """Box found by its own colour: the lowest point of the blob lies on the floor at the near rim.

    Works at any distance the box is in the picture (no depth needed), so it covers the depth sensor's blind zone.
    """
    mask = (_hsv_mask(frame.rgb, model["lo"], model["hi"]) > 0).astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    best = None
    for lab in range(1, count):
        x, y, w, h, area = stats[lab]
        if area >= min_pixels and (best is None or area > best[1]):
            best = (lab, area)
    if best is None:
        return None
    ys, xs = np.nonzero(labels == best[0])
    K = frame.K
    row = int(np.percentile(ys, 97))
    cols = xs[ys >= row - 3]
    u = float(np.mean(cols))
    ray = t_base_cam[:3, :3] @ np.array([(u - K[0, 2]) / K[0, 0], (row - K[1, 2]) / K[1, 1], 1.0])
    origin = t_base_cam[:3, 3]
    if ray[2] > -0.05:
        return None                                   # the lowest blob point is at or above the horizon: not on the floor
    s = (0.0 - origin[2]) / ray[2]
    near_xy = (origin + s * ray)[:2]
    d = near_xy / max(float(np.linalg.norm(near_xy)), 1e-6)
    center = near_xy + d * model["length"] / 2
    return BoxObs(center, 0.0, model["length"], model["width"], model["rim"], int(best[1]), near_xy)
