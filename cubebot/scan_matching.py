"""Two-lidar merge and point-to-line ICP for lidar odometry."""
import math

import numpy as np
from scipy.spatial import cKDTree

from .geometry import rot_z


class Scan:
    """Points in the base frame with per-point unit normals (NaN where unreliable)."""

    def __init__(self, points, normals):
        self.points = points
        self.normals = normals


def _ordered_normals(points, max_gap=0.15):
    n = len(points)
    normals = np.full((n, 2), np.nan)
    if n < 3:
        return normals
    prev, nxt = points[:-2], points[2:]
    tangent = nxt - prev
    length = np.linalg.norm(tangent, axis=1)
    gap = np.maximum(np.linalg.norm(points[1:-1] - prev, axis=1), np.linalg.norm(nxt - points[1:-1], axis=1))
    ok = (length > 1e-6) & (gap < max_gap)
    t = tangent[ok] / length[ok, None]
    normals[1:-1][ok] = np.stack([-t[:, 1], t[:, 0]], axis=1)
    return normals


def scan_to_base(ranges, angle_min, angle_increment, range_min, range_max, mount):
    ranges = np.asarray(ranges, float)
    angles = angle_min + angle_increment * np.arange(len(ranges))
    ok = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    pts = np.stack([ranges * np.cos(angles), ranges * np.sin(angles)], axis=1)
    pts = pts @ rot_z(mount[2]).T + np.array(mount[:2])
    return pts, ok


def build_scan(parts, self_mask=None, max_range=6.0):
    """`parts`: list of (points, valid_mask) already in the base frame, in sweep order."""
    pts_all, normals_all = [], []
    for pts, ok in parts:
        normals = _ordered_normals(pts)
        # Break neighbourhoods across invalid beams so normals never bridge gaps.
        bad = ~ok
        normals[bad] = np.nan
        for shift in (-1, 1):
            normals[np.roll(bad, shift)] = np.nan
        keep = ok & (np.linalg.norm(pts, axis=1) < max_range)
        pts_all.append(pts[keep])
        normals_all.append(normals[keep])
    points = np.concatenate(pts_all) if pts_all else np.zeros((0, 2))
    normals = np.concatenate(normals_all) if normals_all else np.zeros((0, 2))
    if self_mask is not None and len(points):
        (x0, x1), (y0, y1) = self_mask["x"], self_mask["y"]
        inside = (points[:, 0] > x0) & (points[:, 0] < x1) & (points[:, 1] > y0) & (points[:, 1] < y1)
        points, normals = points[~inside], normals[~inside]
    return Scan(points, normals)


class IcpResult:
    def __init__(self, delta, rms, inlier_ratio, degenerate, iterations):
        self.delta = delta
        self.rms = rms
        self.inlier_ratio = inlier_ratio
        self.degenerate = degenerate
        self.iterations = iterations


def icp(reference, current, initial=(0., 0., 0.), max_iter=25, gate=(0.25, 0.04), min_points=40):
    """Pose of `current` scan frame expressed in the `reference` scan frame.

    Point-to-line ICP: minimises normal distances to reference lines. `degenerate`
    flags a geometry that cannot constrain all three DoF (a featureless corridor).
    """
    if len(reference.points) < min_points or len(current.points) < min_points:
        return None
    valid = ~np.isnan(reference.normals[:, 0])
    if valid.sum() < min_points:
        return None
    ref_pts = reference.points[valid]
    ref_nrm = reference.normals[valid]
    tree = cKDTree(ref_pts)
    x, y, th = initial
    rms, ratio, degenerate, it = float("inf"), 0.0, True, 0
    for it in range(1, max_iter + 1):
        c, s = math.cos(th), math.sin(th)
        moved = current.points @ np.array([[c, s], [-s, c]]) + np.array([x, y])
        gate_d = gate[0] + (gate[1] - gate[0]) * min(1.0, it / 10.0)
        dist, idx = tree.query(moved, distance_upper_bound=gate_d)
        hit = np.isfinite(dist)
        if hit.sum() < min_points:
            return None
        p = moved[hit]
        q = ref_pts[idx[hit]]
        n = ref_nrm[idx[hit]]
        e = np.einsum("ij,ij->i", n, p - q)
        # Linearised update (dx, dy, dth) about the current estimate.
        cross = p[:, 0] * n[:, 1] - p[:, 1] * n[:, 0]
        jac = np.column_stack([n[:, 0], n[:, 1], cross])
        # Huber-style weights keep a few bad associations from steering the fit.
        w = 1.0 / np.maximum(1.0, np.abs(e) / 0.02)
        h = (jac * w[:, None]).T @ jac
        g = (jac * w[:, None]).T @ e
        try:
            step = -np.linalg.solve(h + np.eye(3) * 1e-9, g)
        except np.linalg.LinAlgError:
            return None
        # Apply the step in the reference frame: p' = R(dth) p + d.
        dth = step[2]
        cd, sd = math.cos(dth), math.sin(dth)
        x, y = cd * x - sd * y + step[0], sd * x + cd * y + step[1]
        th += dth
        rms = float(math.sqrt(np.mean(e * e)))
        ratio = float(hit.sum()) / len(current.points)
        # Scale the angular term by ~1 m of lever arm so eigenvalues are comparable.
        eig = np.linalg.eigvalsh(h / max(1, hit.sum()))
        degenerate = bool(eig[0] < 0.02)
        if np.linalg.norm(step[:2]) < 1e-4 and abs(dth) < 1e-4:
            break
    return IcpResult((x, y, th), rms, ratio, degenerate, it)
