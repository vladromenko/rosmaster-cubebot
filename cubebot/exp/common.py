"""Shared building blocks of the experiments: perception thread, person tracker + target lock, results store, base class.

Everything that moves the robot goes through `BaseIO.drive` (deadman, lidar guard, acceleration limits), never
straight to /cmd_vel, so the experiments inherit the safety layer of the cube mission.
"""
import contextlib
import datetime
import json
import math
import os
import threading
import time
from dataclasses import dataclass, field

import cv2
import numpy as np
from scipy.optimize import linear_sum_assignment

from ..geometry import wrap
from ..robot_io import MissionAbort
from ..vision import deproject


def expand(path):
    return os.path.expanduser(path)


# --------------------------------------------------------------------------------------- geometry helpers
def box_iou(a, b):
    x0, y0 = max(a[0], b[0]), max(a[1], b[1])
    x1, y1 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, x1 - x0) * max(0.0, y1 - y0)
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def region_depth(depth, x0, y0, x1, y1, lo=0.2, hi=6.0, min_fraction=0.2):
    """Median valid depth (m) of a pixel rectangle, or None when too few pixels carry a measurement."""
    h, w = depth.shape
    x0, x1 = int(max(0, x0)), int(min(w, x1))
    y0, y1 = int(max(0, y0)), int(min(h, y1))
    if x1 - x0 < 2 or y1 - y0 < 2:
        return None
    patch = depth[y0:y1, x0:x1]
    valid = patch[(patch > lo) & (patch < hi)]
    if valid.size < max(6, min_fraction * patch.size):
        return None
    return float(np.median(valid))


def pixel_to_base(u, v, z, K, t_base_cam):
    """Pixel + depth -> base-frame xyz (t_base_cam is the camera *optical* frame pose in base)."""
    p = deproject(u, v, z, K)
    return (t_base_cam @ np.array([p[0], p[1], p[2], 1.0]))[:3]


def appearance(rgb, box):
    """HSV histogram of the torso: identity cue for re-acquiring a person after an occlusion."""
    x0, y0, x1, y1 = [int(v) for v in box]
    w, h = x1 - x0, y1 - y0
    cx0, cx1 = x0 + int(0.2 * w), x1 - int(0.2 * w)
    cy0, cy1 = y0 + int(0.18 * h), y0 + int(0.62 * h)
    H, W = rgb.shape[:2]
    cx0, cx1, cy0, cy1 = max(0, cx0), min(W, cx1), max(0, cy0), min(H, cy1)
    if cx1 - cx0 < 4 or cy1 - cy0 < 4:
        return None
    hsv = cv2.cvtColor(np.ascontiguousarray(rgb[cy0:cy1, cx0:cx1]), cv2.COLOR_RGB2HSV)
    hist = cv2.calcHist([hsv], [0, 1, 2], None, [8, 4, 4], [0, 180, 0, 256, 0, 256])
    return cv2.normalize(hist, hist, 1.0, 0.0, cv2.NORM_L1).flatten().astype(np.float32)


def hist_distance(a, b):
    if a is None or b is None:
        return 0.5
    return float(cv2.compareHist(a, b, cv2.HISTCMP_BHATTACHARYYA))


# --------------------------------------------------------------------------------------- person tracking
@dataclass
class Track:
    id: int
    box: np.ndarray
    score: float
    t: float
    hist: np.ndarray = None
    vel: np.ndarray = field(default_factory=lambda: np.zeros(4))
    hits: int = 1
    misses: int = 0
    seen_t: float = 0.0
    xyz_base: np.ndarray = None          # torso point in the base frame (None: no depth)
    dist: float = None                   # ground-plane distance from the base centre
    bearing: float = None                # atan2(y, x) of the torso point
    mono: bool = False                   # distance came from the box height, not from depth

    def predicted(self, t):
        return self.box + self.vel * max(0.0, min(t - self.t, 1.0))


class PersonTracker:
    """ByteTrack-style tracker: IoU + colour association, a second pass for low-score boxes, constant-velocity prediction."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.tracks = {}
        self._next = 1

    def reset(self):
        self.tracks.clear()

    def _cost(self, track, box, hist, t):
        pred = track.predicted(t)
        iou = box_iou(pred, box)
        c1 = np.array([(pred[0] + pred[2]) / 2, (pred[1] + pred[3]) / 2])
        c2 = np.array([(box[0] + box[2]) / 2, (box[1] + box[3]) / 2])
        height = max(10.0, pred[3] - pred[1])
        reach = height * (0.8 + 0.6 * track.misses)
        if iou < self.cfg["iou_gate"] and np.linalg.norm(c1 - c2) > reach:
            return 9.0
        appearance_cost = hist_distance(track.hist, hist)
        w = self.cfg["hist_weight"]
        return (1 - w) * (1 - iou) + w * appearance_cost

    def update(self, dets, rgb, t):
        """dets: list of (box[4], score). Returns the live tracks (matched this frame or coasting)."""
        cfg = self.cfg
        high = [d for d in dets if d[1] >= cfg["high_score"]]
        low = [d for d in dets if d[1] < cfg["high_score"]]
        hists = {id(d): appearance(rgb, d[0]) for d in high}
        live = list(self.tracks.values())
        matched = set()
        unmatched_tracks = list(live)
        unmatched_high = list(high)
        # pass 1: confident boxes vs every track
        if unmatched_tracks and unmatched_high:
            cost = np.array([[self._cost(tr, d[0], hists[id(d)], t) for d in unmatched_high] for tr in unmatched_tracks])
            rows, cols = linear_sum_assignment(cost)
            keep_t, keep_d = set(range(len(unmatched_tracks))), set(range(len(unmatched_high)))
            for r, c in zip(rows, cols):
                if cost[r, c] <= cfg["accept_cost"]:
                    self._apply(unmatched_tracks[r], unmatched_high[c], hists[id(unmatched_high[c])], t, high)
                    matched.add(unmatched_tracks[r].id)
                    keep_t.discard(r)
                    keep_d.discard(c)
            unmatched_tracks = [unmatched_tracks[i] for i in sorted(keep_t)]
            unmatched_high = [unmatched_high[i] for i in sorted(keep_d)]
        # pass 2: weak boxes may keep an existing track alive through a bad frame (ByteTrack idea)
        if unmatched_tracks and low:
            cost = np.array([[1 - box_iou(tr.predicted(t), d[0]) for d in low] for tr in unmatched_tracks])
            rows, cols = linear_sum_assignment(cost)
            for r, c in zip(rows, cols):
                if cost[r, c] <= 0.6:
                    tr = unmatched_tracks[r]
                    self._apply(tr, low[c], None, t, high, update_hist=False)
                    matched.add(tr.id)
        # new tracks from unexplained confident boxes
        for d in unmatched_high:
            tr = Track(self._next, np.array(d[0], float), d[1], t, hists[id(d)], seen_t=t)
            self._next += 1
            self.tracks[tr.id] = tr
            matched.add(tr.id)
        for tr in list(self.tracks.values()):
            if tr.id not in matched:
                tr.misses += 1
                tr.box = tr.predicted(t)
                tr.t = t
                if t - tr.seen_t > cfg["max_age_s"]:
                    del self.tracks[tr.id]
        return list(self.tracks.values())

    def _apply(self, tr, det, hist, t, all_high, update_hist=True):
        box = np.array(det[0], float)
        dt = max(1e-3, t - tr.t)
        tr.vel = 0.6 * tr.vel + 0.4 * (box - tr.box) / dt if tr.misses == 0 else np.zeros(4)
        tr.box, tr.score, tr.t, tr.seen_t = box, det[1], t, t
        tr.hits += 1
        tr.misses = 0
        # Learn the colour only while nobody else overlaps: otherwise the model would absorb the neighbour.
        alone = all(box_iou(box, o[0]) < 0.15 for o in all_high if o[0] is not det[0])
        if update_hist and hist is not None and alone:
            tr.hist = hist if tr.hist is None else (0.9 * tr.hist + 0.1 * hist).astype(np.float32)


class TargetLock:
    """One chosen person. The lock follows the track id, verifies the colour model and re-acquires only a candidate
    that matches both the model and the last known direction: another person walking by is never adopted."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.clear()

    def clear(self):
        self.id = None
        self.hist = None
        self.last_bearing = 0.0
        self.last_xy = None
        self.last_seen = 0.0
        self.locked_at = 0.0
        self.reacquired = 0
        self.rejected = 0
        self.id_changes = 0

    @property
    def locked(self):
        return self.id is not None

    def lock(self, track, now):
        self.id = track.id
        self.hist = None if track.hist is None else track.hist.copy()
        self.last_seen = self.locked_at = now
        self.last_bearing = track.bearing or 0.0
        self.last_xy = None if track.xyz_base is None else track.xyz_base[:2].copy()

    def update(self, tracks, now):
        """Returns the target track (possibly just re-acquired) or None."""
        by_id = {t.id: t for t in tracks}
        tr = by_id.get(self.id)
        if tr is not None and tr.misses == 0:
            self._seen(tr, now)
            return tr
        if tr is not None and tr.misses > 0:
            # coasting: another confident track that matches the colour at the predicted place wins back the lock
            cand = self._candidate(tracks, now, exclude=self.id)
            if cand is not None:
                self._switch(cand)
                self._seen(cand, now)
                return cand
            return None
        cand = self._candidate(tracks, now, exclude=None)
        if cand is not None:
            self._switch(cand)
            self._seen(cand, now)
            return cand
        return None

    def _switch(self, cand):
        self.id = cand.id
        self.reacquired += 1
        self.id_changes += 1

    def _seen(self, tr, now):
        self.last_seen = now
        if tr.bearing is not None:
            self.last_bearing = tr.bearing
        if tr.xyz_base is not None:
            self.last_xy = tr.xyz_base[:2].copy()
        if tr.hist is not None and self.hist is not None and tr.misses == 0:
            self.hist = (0.97 * self.hist + 0.03 * tr.hist).astype(np.float32)

    def _candidate(self, tracks, now, exclude):
        lost_for = now - self.last_seen
        best, best_d = None, 9.0
        for tr in tracks:
            if tr.id == exclude or tr.misses > 0 or tr.hits < self.cfg["min_hits_to_lock"]:
                continue
            d = hist_distance(self.hist, tr.hist)
            gate = 0.45 + 0.45 * min(lost_for, 6.0)             # the person cannot have turned up at a random place
            if tr.bearing is not None and abs(wrap(tr.bearing - self.last_bearing)) > gate:
                self.rejected += 1
                continue
            if d > self.cfg["reid_max_dist"]:
                self.rejected += 1
                continue
            if d < best_d:
                best, best_d = tr, d
        return best


# --------------------------------------------------------------------------------------- perception thread
@dataclass
class Snapshot:
    stamp: float
    frame: object
    t_base_cam: np.ndarray
    people: list = field(default_factory=list)       # tracked persons (Track)
    objects: list = field(default_factory=list)      # dict(label, score, box, xyz_base, dist)
    hands: list = field(default_factory=list)        # dict(...) see Perception._hands
    poses: list = field(default_factory=list)        # dict(...) see Perception._poses
    processed: float = 0.0
    cycle_ms: float = 0.0


class Perception:
    """Background thread running the MediaPipe models (CPU) on the newest camera frame.

    Models are loaded and run only for the features an experiment asked for (`want`), so an experiment that only
    follows a person does not pay for the hand model."""

    def __init__(self, io, cfg, log=print):
        self.io, self.cfg, self.log = io, cfg, log
        self.ecfg = cfg["exp"]
        self.want = dict(people=False, objects=False, hands=False, pose=False)
        self.crop = None                       # (x0, y0, x1, y1): region where hands/pose are searched (px)
        self.tracker = PersonTracker(self.ecfg["tracker"])
        self.snapshot = None
        self.error = None
        self.hz = 0.0
        self._models = {}
        self._cycle = 0
        self._run = True
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def shutdown(self):
        self._run = False

    def set_want(self, **kw):
        self.want.update(kw)

    # ---- models
    def _mp(self):
        import mediapipe as mp
        from mediapipe.tasks import python as mpp
        from mediapipe.tasks.python import vision
        return mp, mpp, vision

    def _path(self, name):
        return os.path.join(expand(self.ecfg["paths"]["models"]), name)

    def _model(self, key):
        if key in self._models:
            return self._models[key]
        mp, mpp, vision = self._mp()
        d = self.ecfg["detector"]
        base = lambda name: mpp.BaseOptions(model_asset_path=self._path(name))
        if key == "det":
            m = vision.ObjectDetector.create_from_options(vision.ObjectDetectorOptions(
                base_options=base("efficientdet_lite0.tflite"), score_threshold=d["low_score"], max_results=20))
        elif key == "hands":
            m = vision.GestureRecognizer.create_from_options(vision.GestureRecognizerOptions(
                base_options=base("gesture_recognizer.task"), num_hands=2, min_hand_detection_confidence=0.4,
                min_hand_presence_confidence=0.4, min_tracking_confidence=0.4))
        elif key == "pose":
            m = vision.PoseLandmarker.create_from_options(vision.PoseLandmarkerOptions(
                base_options=base("pose_landmarker_lite.task"), num_poses=2, min_pose_detection_confidence=0.4,
                min_pose_presence_confidence=0.4, min_tracking_confidence=0.4))
        else:
            raise KeyError(key)
        self._models[key] = m
        return m

    def new_detector(self):
        """A private ObjectDetector (other threads must not share the one used by the perception loop)."""
        _, mpp, vision = self._mp()
        return vision.ObjectDetector.create_from_options(vision.ObjectDetectorOptions(
            base_options=mpp.BaseOptions(model_asset_path=self._path("efficientdet_lite0.tflite")),
            score_threshold=self.ecfg["detector"]["low_score"], max_results=20))

    def _mp_image(self, rgb):
        mp, _, _ = self._mp()
        return mp.Image(image_format=mp.ImageFormat.SRGB, data=np.ascontiguousarray(rgb))

    # ---- main loop
    def _loop(self):
        last_stamp, times = 0.0, []
        while self._run:
            frame = self.io._latest_frame()
            if frame is None or frame.stamp == last_stamp or not any(self.want.values()):
                time.sleep(0.03)
                continue
            last_stamp = frame.stamp
            t0 = time.monotonic()
            try:
                snap = self._process(frame)
                with self._lock:
                    self.snapshot = snap
                self.error = None
            except Exception as exc:                                  # a bad frame must not kill the thread
                self.error = repr(exc)
                self.log("perception error: %r" % (exc,))
                time.sleep(0.2)
            times.append(time.monotonic())
            times = [x for x in times if times[-1] - x < 3.0]
            self.hz = (len(times) - 1) / max(1e-3, times[-1] - times[0]) if len(times) > 1 else 0.0
            if self.snapshot is not None:
                self.snapshot.cycle_ms = (time.monotonic() - t0) * 1000

    def latest(self, max_age=None):
        snap = self.snapshot
        if snap is None:
            return None
        if max_age is not None and time.monotonic() - snap.processed > max_age:
            return None
        return snap

    def _process(self, frame):
        t = time.monotonic()
        t_cam = self.io.arm.camera_pose(self.io.arm_command())
        snap = Snapshot(frame.stamp, frame, t_cam, processed=t)
        rgb = frame.rgb
        d = self.ecfg["detector"]
        self._cycle += 1
        run_det = (self.want["people"] or self.want["objects"]) and (
            self._cycle % max(1, d["people_every"]) == 0 or self.snapshot is None)
        if run_det:
            dets = self._model("det").detect(self._mp_image(rgb)).detections
            persons, objects = [], []
            for det in dets:
                c = det.categories[0]
                bb = det.bounding_box
                box = np.array([bb.origin_x, bb.origin_y, bb.origin_x + bb.width, bb.origin_y + bb.height], float)
                if c.category_name == "person":
                    persons.append((box, float(c.score)))
                elif self.want["objects"] and c.score >= d["object_score"]:
                    objects.append(self._object(c.category_name, float(c.score), box, frame, t_cam))
            if self.want["people"]:
                tracks = self.tracker.update(persons, rgb, t)
                for tr in tracks:
                    if tr.misses == 0:
                        self._localise(tr, frame, t_cam)
                snap.people = tracks
            snap.objects = objects
        elif self.snapshot is not None:                  # reuse the last detection result between detector runs
            prev = self.snapshot
            snap.people, snap.objects = prev.people, prev.objects
        if self.want["hands"]:
            snap.hands = self._hands(frame, t_cam)
        if self.want["pose"]:
            snap.poses = self._poses(frame, t_cam)
        return snap

    def _object(self, label, score, box, frame, t_cam):
        cx, cy = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
        z = region_depth(frame.depth, cx - 0.2 * (box[2] - box[0]), cy - 0.2 * (box[3] - box[1]),
                         cx + 0.2 * (box[2] - box[0]), cy + 0.2 * (box[3] - box[1]), lo=0.2, hi=3.0)
        xyz = None if z is None else pixel_to_base(cx, cy, z, frame.K, t_cam)
        width = None if z is None else float((box[2] - box[0]) * z / frame.K[0, 0])
        return dict(label=label, score=score, box=box, xyz_base=xyz, width_m=width,
                    dist=None if xyz is None else float(np.hypot(xyz[0], xyz[1])))

    def _localise(self, tr, frame, t_cam):
        x0, y0, x1, y1 = tr.box
        w, h = x1 - x0, y1 - y0
        cx = (x0 + x1) / 2
        z = region_depth(frame.depth, cx - 0.15 * w, y0 + 0.25 * h, cx + 0.15 * w, y0 + 0.55 * h)
        K = frame.K
        if z is not None:
            tr.xyz_base = pixel_to_base(cx, y0 + 0.4 * h, z, K, t_cam)
            tr.mono = False
        else:                                         # monocular fallback from the apparent height (1.7 m person)
            if h < 40:
                tr.xyz_base = None
                tr.dist = tr.bearing = None
                return
            z = K[1, 1] * 1.7 / h
            tr.xyz_base = pixel_to_base(cx, y0 + 0.4 * h, z, K, t_cam)
            tr.mono = True
        tr.dist = float(np.hypot(tr.xyz_base[0], tr.xyz_base[1]))
        tr.bearing = float(math.atan2(tr.xyz_base[1], tr.xyz_base[0]))

    def _roi(self, frame):
        crop = self.crop
        h, w = frame.rgb.shape[:2]
        if not crop:
            return 0, 0, w, h
        x0, y0, x1, y1 = [int(v) for v in crop]
        return max(0, x0), max(0, y0), min(w, x1), min(h, y1)

    def _hands(self, frame, t_cam):
        x0, y0, x1, y1 = self._roi(frame)
        sub = frame.rgb[y0:y1, x0:x1]
        if sub.shape[0] < 40 or sub.shape[1] < 40:
            return []
        scale = 1.0
        if min(sub.shape[:2]) < 400:                 # distant hands are tiny: enlarge the crop for the hand model
            scale = 400.0 / min(sub.shape[:2])
            sub = cv2.resize(sub, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
        # MediaPipe only handles square regions correctly: letterbox the crop and map the landmarks back
        sh, sw = sub.shape[:2]
        side = max(sh, sw)
        oy, ox = (side - sh) // 2, (side - sw) // 2
        square = np.zeros((side, side, 3), np.uint8)
        square[oy:oy + sh, ox:ox + sw] = sub
        res = self._model("hands").recognize(self._mp_image(square))
        out = []
        for i, lms in enumerate(res.hand_landmarks):
            pts = np.array([[x0 + (p.x * side - ox) / scale, y0 + (p.y * side - oy) / scale] for p in lms])
            label, score = "None", 0.0
            if res.gestures and i < len(res.gestures) and res.gestures[i]:
                label, score = res.gestures[i][0].category_name, float(res.gestures[i][0].score)
            handed = res.handedness[i][0].category_name if res.handedness and i < len(res.handedness) else "?"
            c = pts[[0, 5, 9, 13, 17]].mean(axis=0)
            z = region_depth(frame.depth, c[0] - 8, c[1] - 8, c[0] + 8, c[1] + 8, lo=0.2, hi=4.0, min_fraction=0.1)
            xyz = None if z is None else pixel_to_base(c[0], c[1], z, frame.K, t_cam)
            out.append(dict(pts=pts, label=label, score=score, handed=handed, depth=z, xyz_base=xyz))
        return out

    def _poses(self, frame, t_cam):
        x0, y0, x1, y1 = self._roi(frame)
        sub = frame.rgb[y0:y1, x0:x1]
        if sub.shape[0] < 60 or sub.shape[1] < 40:
            return []
        sh, sw = sub.shape[:2]
        side = max(sh, sw)
        oy, ox = (side - sh) // 2, (side - sw) // 2
        square = np.zeros((side, side, 3), np.uint8)
        square[oy:oy + sh, ox:ox + sw] = sub
        res = self._model("pose").detect(self._mp_image(square))
        out = []
        for lms in res.pose_landmarks:
            pts = np.array([[x0 + p.x * side - ox, y0 + p.y * side - oy, p.visibility] for p in lms])
            joints = {}
            for name, idx in (("l_shoulder", 11), ("r_shoulder", 12), ("l_elbow", 13), ("r_elbow", 14), ("l_wrist", 15),
                              ("r_wrist", 16), ("l_hip", 23), ("r_hip", 24), ("nose", 0)):
                u, v, vis = pts[idx]
                z = region_depth(frame.depth, u - 6, v - 6, u + 6, v + 6, lo=0.2, hi=4.5, min_fraction=0.1)
                joints[name] = dict(uv=(float(u), float(v)), vis=float(vis),
                                    xyz=None if z is None else pixel_to_base(u, v, z, frame.K, t_cam))
            out.append(joints)
        return out


# --------------------------------------------------------------------------------------- results
class ResultStore:
    """Trials are appended to JSON-lines files: results/<experiment>/trials.jsonl (+ videos next to them)."""

    def __init__(self, cfg):
        self.root = expand(cfg["exp"]["paths"]["results"])
        self._lock = threading.Lock()

    def dir(self, exp):
        d = os.path.join(self.root, exp)
        os.makedirs(d, exist_ok=True)
        return d

    def add(self, exp, record):
        record = dict(record)
        record.setdefault("time", datetime.datetime.now().isoformat(timespec="seconds"))
        with self._lock, open(os.path.join(self.dir(exp), "trials.jsonl"), "a") as fh:
            fh.write(json.dumps(record, default=_json) + "\n")
        return record

    def load(self, exp, limit=200):
        path = os.path.join(self.dir(exp), "trials.jsonl")
        if not os.path.exists(path):
            return []
        rows = []
        with open(path) as fh:
            for line in fh:
                try:
                    rows.append(json.loads(line))
                except ValueError:
                    pass
        return rows[-limit:]


def _json(o):
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    if isinstance(o, np.ndarray):
        return o.tolist()
    return str(o)


class Recorder:
    """MJPEG .avi of the annotated camera view while an experiment runs."""

    def __init__(self, directory, name, fps=8.0):
        self.path = os.path.join(directory, "%s_%s.avi" % (name, time.strftime("%Y%m%d_%H%M%S")))
        self.fps, self.writer, self._last = fps, None, 0.0

    def write(self, bgr):
        now = time.monotonic()
        if now - self._last < 1.0 / self.fps:
            return
        self._last = now
        if self.writer is None:
            h, w = bgr.shape[:2]
            self.writer = cv2.VideoWriter(self.path, cv2.VideoWriter_fourcc(*"MJPG"), self.fps, (w, h))
        self.writer.write(bgr)

    def close(self):
        if self.writer is not None:
            self.writer.release()
            self.writer = None


# --------------------------------------------------------------------------------------- experiment base class
class Context:
    """What an experiment gets from the console."""

    def __init__(self, io, cfg, perception, store, log, console=None):
        self.io, self.cfg, self.perception, self.store, self.log, self.console = io, cfg, perception, store, log, console
        self.arm = io.arm


@contextlib.contextmanager
def base_limits(io, **limits):
    """Temporarily raise the base speed/acceleration limits (the shaper reads cfg['base'] every cycle)."""
    cfg = io.shaper.cfg
    saved = {k: cfg[k] for k in limits}
    cfg.update(limits)
    try:
        yield
    finally:
        cfg.update(saved)
        io.shaper.reset()


class Experiment:
    key = "exp"
    title = "experiment"

    def __init__(self, ctx):
        self.ctx = ctx
        self.io, self.cfg, self.log = ctx.io, ctx.cfg, ctx.log
        self.ecfg = ctx.cfg["exp"]
        self.state = "idle"
        self.message = ""
        self.result = None
        self.metrics = {}
        self.running = False
        self.params = {}
        self._thread = None
        self._recorder = None
        self._stop = threading.Event()
        self._saved_runtime = None
        self.started_at = None

    # ---- lifecycle
    def say(self, text):
        self.message = text
        self.log("[%s] %s" % (self.key, text))

    def start(self, params=None):
        if self.running:
            return False
        self.params = dict(params or {})
        self.running = True
        self.result = None
        self.state = "starting"
        self._stop.clear()
        self.started_at = time.monotonic()
        self._recorder = Recorder(self.ctx.store.dir(self.key), self.key)
        self._thread = threading.Thread(target=self._main, daemon=True)
        self._thread.start()
        return True

    def stop(self, reason="operator stop"):
        self._stop.set()
        self.io.request_abort(reason)

    def _main(self):
        io = self.io
        try:
            io.abort_reason, io.started = None, None
            io.started = io.now()
            self.setup()
            self.run()
            self.state = self.state if self.state in ("done", "failed") else "done"
        except MissionAbort as exc:
            self.state, self.result = "stopped", "stopped: %s" % exc
        except Exception as exc:
            import traceback
            self.log(traceback.format_exc())
            self.state, self.result = "failed", "error: %r" % (exc,)
        finally:
            try:
                io.stop_base()
                self.teardown()                       # while the abort is still latched: nothing can drive meanwhile
                io.stop_base()
            except Exception as exc:
                self.log("teardown error: %r" % (exc,))
            finally:
                io.abort_reason = None
                if self._saved_runtime is not None:
                    self.cfg["mission"]["max_runtime_s"], self._saved_runtime = self._saved_runtime, None
            if self._recorder is not None:
                self._recorder.close()
                self.metrics["video"] = self._recorder.path
            self.running = False
            self.log("[%s] finished: %s %s" % (self.key, self.state, self.result or ""))

    def setup(self):
        pass

    def run(self):
        raise NotImplementedError

    def teardown(self):
        pass

    # ---- helpers
    def sleep(self, seconds):
        """Wait with the full abort checks (stop button, stop file, battery, runtime)."""
        self.io.wait(seconds)
        if self._stop.is_set():
            raise MissionAbort("operator stop")

    def allow_runtime(self, seconds):
        if self._saved_runtime is None:
            self._saved_runtime = self.cfg["mission"]["max_runtime_s"]
        self.cfg["mission"]["max_runtime_s"] = max(self.cfg["mission"]["max_runtime_s"], seconds)

    def people_view(self):
        self.io.move_arm(list(self.ecfg["people_pose"]))
        self.io.wait(0.4)

    def status(self):
        return dict(key=self.key, title=self.title, state=self.state, message=self.message, running=self.running,
                    result=self.result, metrics=self.metrics, params=self.params,
                    elapsed=None if self.started_at is None or not self.running else round(time.monotonic() - self.started_at, 1))

    def overlay(self, img):
        """Draw experiment annotations on a BGR frame."""
        return img

    def record(self, bgr):
        if self.running and self._recorder is not None:
            self._recorder.write(bgr)


def put_text(img, text, org, color=(255, 255, 255), scale=0.55, thick=1):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thick + 2, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)


def draw_people(img, tracks, target_id=None):
    for tr in tracks:
        x0, y0, x1, y1 = [int(v) for v in tr.box]
        is_target = tr.id == target_id
        color = (0, 255, 0) if is_target else (200, 160, 60)
        if tr.misses > 0:
            color = (0, 140, 255)
        cv2.rectangle(img, (x0, y0), (x1, y1), color, 3 if is_target else 1)
        label = "#%d%s" % (tr.id, " TARGET" if is_target else "")
        if tr.dist is not None:
            label += " %.2f m%s" % (tr.dist, "~" if tr.mono else "")
        put_text(img, label, (x0, max(14, y0 - 6)), color)
