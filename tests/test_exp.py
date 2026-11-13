"""Unit tests of the experiment logic that does not need the robot (they do NOT replace the physical trials)."""
import numpy as np

from cubebot import config
from cubebot.exp.assistant import parse_command
from cubebot.exp.common import PersonTracker, TargetLock, Track, box_iou
from cubebot.exp.follow import FollowRunner
from cubebot.exp.gesture import GestureAnalyzer
from cubebot.exp.navigation import find_frontiers
from cubebot.exp.retrieve import resolve_target

CFG = config.load()
E = CFG["exp"]


def make_hand(kind, wrist=(300, 300), scale=60.0, pdir=(-1, 0), curl=1.9, label="None"):
    pts = np.zeros((21, 2))
    pts[0] = wrist
    pts[9] = (wrist[0], wrist[1] - scale)
    pts[5] = (wrist[0] - 10, wrist[1] - scale)
    ext = {"palm": (1, 1, 1, 1), "point": (1, 0, 0, 0), "other": (0, 0, 0, 0), "thumb": (0, 0, 0, 0)}[kind]
    for finger, ((tip, pip), e) in enumerate(zip(((8, 6), (12, 10), (16, 14), (20, 18)), ext)):
        pts[pip] = (wrist[0] + finger * 8, wrist[1] - 0.8 * scale)
        far = curl * scale if e else 0.5 * scale
        pts[tip] = (wrist[0] + finger * 8, wrist[1] - far)
    if kind == "point":
        pts[8] = pts[5] + np.array(pdir, float) * 80
    return dict(pts=pts, label="Thumb_Up" if kind == "thumb" else label, score=0.9, handed="Right", depth=None, xyz_base=None)


def feed(an, frames, dt=0.15, t0=100.0):
    events = []
    for i, f in enumerate(frames):
        ev = an.update([] if f is None else [f], t0 + i * dt)
        if ev:
            events.append((i, ev))
    return events


def test_stop_needs_hold_and_fires_once():
    an = GestureAnalyzer(E["gesture"])
    events = feed(an, [make_hand("palm")] * 30)
    assert [e for _, e in events] == ["stop"]           # held for 4.5 s: still exactly one command
    assert events[0][0] >= 4                            # not on the first frames


def test_single_frame_is_not_a_gesture():
    an = GestureAnalyzer(E["gesture"])
    assert feed(an, [make_hand("palm"), None, None, None, None, None]) == []


def test_latch_releases_after_hand_leaves():
    an = GestureAnalyzer(E["gesture"])
    seq = [make_hand("palm")] * 10 + [None] * 10 + [make_hand("palm")] * 10
    assert [e for _, e in feed(an, seq)] == ["stop", "stop"]


def test_point_left_right():
    an = GestureAnalyzer(E["gesture"])
    assert [e for _, e in feed(an, [make_hand("point", pdir=(-1, 0.1))] * 10)] == ["left"]
    an = GestureAnalyzer(E["gesture"])
    assert [e for _, e in feed(an, [make_hand("point", pdir=(1, 0.1))] * 10)] == ["right"]
    an = GestureAnalyzer(E["gesture"])
    assert feed(an, [make_hand("point", pdir=(0.1, -1))] * 10) == []     # pointing up is no command


def test_thumb_resume():
    an = GestureAnalyzer(E["gesture"])
    assert [e for _, e in feed(an, [make_hand("thumb")] * 10)] == ["resume"]


def test_push_away_by_growth():
    an = GestureAnalyzer(E["gesture"])
    seq = [make_hand("palm", scale=s) for s in (50, 53, 57, 62, 68, 75)]
    assert "away" in [e for _, e in feed(an, seq, dt=0.15)]


def test_beckon_by_curl_cycles():
    an = GestureAnalyzer(E["gesture"])
    curls = [1.9, 1.9, 1.0, 1.0, 1.9, 1.9, 1.0, 1.0, 1.9, 1.9]
    seq = [make_hand("palm", curl=1.9) if c > 1.5 else make_hand("other") for c in curls]
    assert "come" in [e for _, e in feed(an, seq, dt=0.15)]


def test_waving_hand_is_not_beckoning():
    an = GestureAnalyzer(E["gesture"])
    seq = [make_hand("palm", wrist=(200 + 150 * (i % 2), 300)) for i in range(12)]
    assert "come" not in [e for _, e in feed(an, seq)]


def test_tracker_keeps_identity_when_people_cross():
    tr = PersonTracker(E["tracker"])
    ids = {}
    t = 0.0
    for step in range(16):
        a = [100 + step * 12, 100, 200 + step * 12, 400]
        b = [420 - step * 12, 100, 520 - step * 12, 400]
        img = np.zeros((480, 640, 3), np.uint8)
        for box, col in ((a, (200, 30, 30)), (b, (30, 30, 200))):     # colours follow the people: A red, B blue
            img[box[1]:box[3], box[0]:box[2]] = col
        tracks = tr.update([(np.array(a, float), 0.9), (np.array(b, float), 0.9)], img, t)
        t += 0.15
        for trk in tracks:
            ids.setdefault(trk.id, []).append((trk.box[0] + trk.box[2]) / 2)
    assert len(ids) == 2                                # no new identities created while crossing
    for xs in ids.values():
        assert np.abs(np.diff(xs)).max() < 40           # nobody jumped to the other person


def test_lock_rejects_stranger_and_reacquires_owner():
    lock = TargetLock(E["tracker"])
    red = np.zeros(8 * 4 * 4, np.float32)
    red[3] = 1
    blue = np.zeros(8 * 4 * 4, np.float32)
    blue[60] = 1
    mine = Track(1, np.array([100., 50, 200, 400]), 0.9, 0.0, red, hits=10, seen_t=0.0)
    mine.bearing, mine.xyz_base = 0.1, np.array([1.2, 0.1, 1.0])
    lock.lock(mine, 0.0)
    stranger = Track(2, np.array([120., 50, 220, 400]), 0.9, 5.0, blue, hits=10, seen_t=5.0)
    stranger.bearing, stranger.xyz_base = 0.1, np.array([1.2, 0.1, 1.0])
    assert lock.update([stranger], 5.0) is None and lock.rejected >= 1
    owner = Track(3, np.array([110., 50, 210, 400]), 0.9, 6.0, red.copy(), hits=10, seen_t=6.0)
    owner.bearing, owner.xyz_base = 0.15, np.array([1.2, 0.15, 1.0])
    got = lock.update([stranger, owner], 6.0)
    assert got is owner and lock.reacquired == 1


class _IO:
    def __init__(self):
        self.guard = type("G", (), {"_free_distance": staticmethod(lambda pts, d, reach: reach)})()

    def scan_points(self):
        return None

    def pose(self):
        return (0, 0, 0)


class _Ctx:
    def __init__(self):
        self.io, self.cfg = _IO(), CFG


def person(x, y, mono=False):
    t = Track(1, np.zeros(4), 0.9, 0.0)
    t.xyz_base, t.mono = np.array([x, y, 1.0]), mono
    return t


def test_follow_control_law():
    r = FollowRunner(_Ctx())
    vx, vy, wz = r.twist(person(2.5, 0.0))
    assert vx > 0.1 and abs(vy) < 1e-6 and abs(wz) < 1e-6          # too far: drive straight ahead
    assert r.twist(person(1.2, 0.0)) == (0.0, 0.0, 0.0)             # at the stand-off: hold still
    vx, vy, wz = r.twist(person(0.5, 0.0))
    assert vx < 0                                                    # too close: back off
    vx, vy, wz = r.twist(person(2.0, 0.8))
    assert wz > 0 and vy > 0                                         # person on the left: turn left and strafe left
    vx, vy, wz = r.twist(person(2.0, -1.5))
    assert wz < 0 and abs(vx) < 0.05                                 # large bearing: turn first
    assert max(abs(v) for v in r.twist(person(9.0, 0.0))[:2]) <= E["follow"]["max_vx"] + 1e-9


def test_follow_stops_without_fresh_measurement():
    r = FollowRunner(_Ctx())
    t = person(2.5, 0.0)
    t.hits, t.misses, t.seen_t, t.hist = 5, 0, 10.0, None
    t.dist, t.bearing = 2.5, 0.0
    r.lock.lock(t, 10.0)
    r.state = r.FOLLOW

    class Snap:
        people = [t]
        frame = type("F", (), {"rgb": np.zeros((480, 640, 3))})()

    assert r.tick(Snap, 10.1)[0] > 0                    # fresh: moves
    out = r.tick(Snap, 11.5)                            # same data 1.4 s later: stale, so no translation
    assert out[0] == 0.0 and out[1] == 0.0 and r.state == r.LOST


def test_frontiers_on_a_simple_map():
    grid = np.full((60, 60), -1, np.int8)
    grid[20:40, 10:40] = 0                              # a known free room
    grid[19, 10:40] = grid[40, 10:40] = 100
    grid[20:40, 10] = 100                               # wall on the left; the right side stays open to unknown
    f = find_frontiers(grid, 0.05, (0.0, 0.0), (1.0, 1.0), E["nav"])
    assert f and abs(f[0]["xy"][0] - 1.95) < 0.3        # the opening is on the right (x ~ 2.0 m)


def test_command_parser():
    p = parse_command("Найди кубик, принеси его мне и отдай в руку", E)
    assert p["ok"] and p["object"] == "cube" and p["to_person"] and p["handover"] == "hand"
    p = parse_command("найди бутылку", E)
    assert p["ok"] and p["object"] == "bottle" and not p["to_person"]
    assert parse_command("сделай что-нибудь", E)["ok"] is False
    assert parse_command("стоп", E)["intent"] == "stop"
    assert resolve_target("принеси чашку", E) == "cup"


def test_box_iou():
    assert box_iou([0, 0, 10, 10], [0, 0, 10, 10]) == 1.0
    assert box_iou([0, 0, 10, 10], [20, 20, 30, 30]) == 0.0


def _simulate_follow(person_speed, turn_at=None, seconds=40.0, latency=0.25, vmax=0.28):
    """Closed loop: person walks, the robot sees it with `latency`, accel-limited holonomic base."""
    import math
    from collections import deque
    E["follow"]["max_vx"] = vmax
    r = FollowRunner(_Ctx())
    robot = np.array([0.0, 0.0, 0.0])                  # x, y, yaw in the world
    person = np.array([2.2, 0.0])
    heading = 0.0
    vel = np.zeros(3)
    buf = deque()
    p_seen, r_seen = person.copy(), robot.copy()
    dt, t = 0.05, 0.0
    errors = []
    while t < seconds:
        if turn_at is not None and abs(t - turn_at) < dt / 2:
            heading += math.pi / 2
        person = person + person_speed * dt * np.array([math.cos(heading), math.sin(heading)]) if t < seconds - 8 else person
        buf.append((t, person.copy(), robot.copy()))
        while buf and buf[0][0] < t - latency:         # delayed measurement
            _, p_seen, r_seen = buf.popleft()
        c, s = math.cos(r_seen[2]), math.sin(r_seen[2])
        rel = np.array([c * (p_seen[0] - r_seen[0]) + s * (p_seen[1] - r_seen[1]),
                        -s * (p_seen[0] - r_seen[0]) + c * (p_seen[1] - r_seen[1])])
        cmd = np.array(r.twist(person_track(rel[0], rel[1])))
        lim = np.array([vmax, E["follow"]["max_vy"], E["follow"]["max_wz"]])
        cmd = np.clip(cmd, -lim, lim)
        step = np.array([E["follow"]["accel"], E["follow"]["accel"], E["follow"]["angular_accel"]]) * dt
        vel += np.clip(cmd - vel, -step, step)
        c, s = math.cos(robot[2]), math.sin(robot[2])
        robot += dt * np.array([c * vel[0] - s * vel[1], s * vel[0] + c * vel[1], vel[2]])
        t += dt
        d = float(np.hypot(*(person - robot[:2])))
        errors.append(d)
    return np.array(errors)


def person_track(x, y):
    t = Track(1, np.zeros(4), 0.9, 0.0)
    t.xyz_base = np.array([x, y, 1.0])
    return t


def test_follow_loop_converges_and_does_not_oscillate():
    err = _simulate_follow(person_speed=0.20, seconds=40.0)
    standoff = E["follow"]["standoff_m"]
    tail = err[-int(6 / 0.05):]
    assert abs(tail.mean() - standoff) < 0.2 and tail.std() < 0.05      # settles at the stand-off, no hunting
    assert err.min() > 0.8                                              # never closes in dangerously


def test_follow_loop_survives_a_90_degree_turn():
    err = _simulate_follow(person_speed=0.20, turn_at=15.0, seconds=45.0)
    assert err.min() > 0.7 and err.max() < 3.5
    assert abs(err[-int(6 / 0.05):].mean() - E["follow"]["standoff_m"]) < 0.25
