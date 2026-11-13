"""Live data and calculators behind the "how the experiments work" page (everything here is the robot's real data/code).

/api/exp/live   one JSON per poll: poses, arm, scan, perception, follow/gesture state (+ histories since `since`)
/api/exp/fk     forward kinematics of the real arm model for given servo angles
/api/exp/ik     the real IK solver (arm_model.ik) for a target point
/api/exp/reach  reachable (x, z) cells of the arm for an approach pitch (cached, computed in the background)
/api/exp/pick   back-projection of a camera pixel with the live depth into camera and base frames
/api/exp/nav    SLAM/Nav2 state: map->odom transform, pose on the map, waypoints, trajectory
"""
import math
import threading
import time

import numpy as np
from fastapi import Body
from fastapi.staticfiles import StaticFiles

from ..exp.common import region_depth


def m4(t):
    return np.asarray(t, float).round(5).tolist()


class Live:
    def __init__(self, console):
        self.c = console
        self._reach = {}
        self._reach_lock = threading.Lock()

    # ---------------------------------------------------------------- live telemetry
    def live(self, since=0.0):
        c, io, arm = self.c, self.c.io, self.c.arm
        now = time.monotonic()
        servo = io.measured_arm() or io.arm_command()
        f = arm.fk(servo)
        scan = io.scan_points()
        pts = [] if scan is None else np.asarray(scan)[:: max(1, len(scan) // 360)].round(3).tolist()
        mgr = c.exp
        runner = mgr.follow_runner()
        analyzer = mgr.analyzer()
        snap = mgr.perception.latest(max_age=2.0)
        out = dict(
            t=now, mode=c.mode, dry_run=c.dry_run, pose=[round(v, 4) for v in io.pose()],
            cmd=[round(v, 4) for v in io.last_output], raw=dict(
                twist=[round(v, 4) for v in io.raw["twist"]], wheel=[round(v, 4) for v in io.raw["wheel"]],
                gyro_wz=round(io.raw["gyro_wz"], 4), gyro_yaw=round(io.raw["gyro_yaw"], 4),
                bias=round(io.odometry._gyro_bias, 5)) if hasattr(io, "raw") else None,
            odom_stats=io.odometry.stats, guard=io.guard.last_block,
            arm=dict(cmd=[round(v, 1) for v in io.arm_command()], measured=None if io.measured_arm() is None else [round(v, 1) for v in io.measured_arm()],
                     tcp=[round(float(v), 4) for v in arm.tcp_position(servo)]),
            fk=dict(servo=[round(float(v), 1) for v in servo], camera=m4(f["camera"]), tcp=m4(f["tcp"]),
                    frames=[m4(t) for t in f["frames"]]),
            scan=pts, battery=c.gauge.state().get("volts"),
            cfg={k: c.cfg["exp"][k] for k in ("follow", "gesture", "assistant", "nav", "tracker")},
            camera=None, preview=dict(mgr.preview_want), perception=dict(hz=round(mgr.perception.hz, 1), error=mgr.perception.error),
            people=[], objects=[], hands=[], poses=[],
            exps={k: dict(state=e.state, running=e.running, message=e.message, result=e.result, metrics=_light(e.metrics))
                  for k, e in mgr.exps.items()}, active=None if mgr.active is None or not mgr.active.running else mgr.active.key)
        frame = io._latest_frame()
        if frame is not None:
            out["camera"] = dict(K=np.asarray(frame.K).round(3).tolist(), size=[int(frame.rgb.shape[1]), int(frame.rgb.shape[0])],
                                 age=round(time.monotonic() - frame.stamp, 2))
        if snap is not None:
            tgt = runner.lock.id if runner is not None else None
            for t in snap.people:
                out["people"].append(dict(id=t.id, box=[round(float(v)) for v in t.box], score=round(float(t.score), 2), hits=t.hits,
                                          misses=t.misses, dist=None if t.dist is None else round(float(t.dist), 3),
                                          bearing=None if t.bearing is None else round(float(t.bearing), 4), mono=t.mono,
                                          xyz=None if t.xyz_base is None else np.asarray(t.xyz_base).round(3).tolist(),
                                          target=t.id == tgt, age=round(now - t.seen_t, 2)))
            for o in snap.objects:
                out["objects"].append(dict(label=o["label"], score=round(o["score"], 2), box=[round(float(v)) for v in o["box"]],
                                           xyz=None if o["xyz_base"] is None else np.asarray(o["xyz_base"]).round(3).tolist()))
            for h in snap.hands:
                out["hands"].append(dict(pts=np.asarray(h["pts"]).round(1).tolist(), label=h["label"], handed=h["handed"],
                                         depth=h["depth"]))
            for p in snap.poses:
                out["poses"].append({k: dict(uv=[round(v, 1) for v in j["uv"]], vis=round(j["vis"], 2),
                                             xyz=None if j["xyz"] is None else np.asarray(j["xyz"]).round(3).tolist())
                                     for k, j in p.items()})
        if runner is not None:
            tgt = runner.target
            out["follow"] = dict(state=runner.state, cmd=[round(v, 3) for v in runner.cmd], note=runner.note, losses=runner.losses,
                                 target_id=runner.lock.id, reacquired=runner.lock.reacquired, rejected=runner.lock.rejected,
                                 standoff=c.cfg["exp"]["follow"]["standoff_m"],
                                 hist=[[round(x[0], 2)] + [None if v is None else round(v, 3) if isinstance(v, float) else v for v in x[1:]]
                                       for x in list(runner.history) if x[0] > since])
        if analyzer is not None:
            out["gesture"] = dict(debug=analyzer.debug, latched=analyzer.latched, cooldown=max(0.0, round(analyzer.cooldown_until - now, 2)),
                                  events=[[round(t, 2), n] for t, n in analyzer.event_log],
                                  hist=[[round(x[0], 2)] + [None if v is None else round(v, 3) if isinstance(v, float) else v for v in x[1:]]
                                        for x in list(analyzer.history) if x[0] > since],
                                  cfg=c.cfg["exp"]["gesture"])
        return out

    # ---------------------------------------------------------------- calculators
    def fk(self, servo):
        arm = self.c.arm
        f = arm.fk([float(v) for v in servo])
        return dict(frames=[m4(t) for t in f["frames"]], tcp=m4(f["tcp"]), camera=m4(f["camera"]),
                    tcp_xyz=np.asarray(f["tcp"][:3, 3]).round(4).tolist(),
                    pitch_down_deg=round(math.degrees(arm.approach_pitch_down(servo)), 2))

    def ik(self, target, pitch, seed=None):
        arm = self.c.arm
        seed = [float(v) for v in (seed or self.c.io.arm_command())]
        t0 = time.perf_counter()
        sol, err = arm.ik(np.asarray(target, float), float(pitch), seed)
        ms = (time.perf_counter() - t0) * 1000
        if sol is None:
            return dict(ok=False, residual_m=None if not np.isfinite(err) else round(float(err), 4), ms=round(ms, 1))
        servo = [float(v) for v in sol] + [float(seed[5])]
        f = arm.fk(servo)
        return dict(ok=True, servo=[round(v, 2) for v in servo], residual_m=round(float(err), 5), ms=round(ms, 1),
                    tcp=np.asarray(f["tcp"][:3, 3]).round(4).tolist(), pitch_down_deg=round(math.degrees(arm.approach_pitch_down(servo)), 2),
                    collision_free=bool(arm.collision_free(servo, self.c.cfg["arm"]["floor_margin_m"])),
                    within_limits=bool(arm.within_limits(servo)), jacobian=np.asarray(arm.tcp_jacobian_deg(servo)).round(5).tolist(),
                    frames=[m4(t) for t in f["frames"]])

    def reach(self, pitch, y=0.0):
        """Reachable (x, z) cells; computed once per (pitch, y) in a background thread."""
        key = (int(pitch), round(float(y), 2))
        with self._reach_lock:
            entry = self._reach.get(key)
            if entry is None:
                entry = self._reach[key] = dict(ready=False, points=[])
                threading.Thread(target=self._reach_job, args=(key, entry), daemon=True).start()
        return dict(ready=entry["ready"], points=entry["points"], done=entry.get("done", 0), total=entry.get("total", 0))

    def _reach_job(self, key, entry):
        arm, c = self.c.arm, self.c
        seed = list(c.cfg["arm"]["safe_pose"])
        xs, zs = np.arange(0.06, 0.46, 0.02), np.arange(0.01, 0.72, 0.03)
        entry["total"] = len(xs) * len(zs)
        pts, col_seed = [], seed
        for x in xs:                                   # warm start from the neighbouring cell: single-seed IK then converges
            last = col_seed
            for z in zs:
                sol, _ = arm.ik(np.array([x, key[1], z]), key[0], last, quick=True)
                entry["done"] = entry.get("done", 0) + 1
                if sol is not None:
                    pts.append([round(float(x), 3), round(float(z), 3)])
                    last = list(sol) + [seed[5]]
                    if z == zs[0]:
                        col_seed = last
        entry["points"], entry["ready"] = pts, True

    def pick(self, u, v):
        io, c = self.c.io, self.c
        frame = io._latest_frame()
        if frame is None:
            return dict(ok=False, error="нет кадра камеры")
        z = region_depth(frame.depth, u - 3, v - 3, u + 4, v + 4, lo=0.2, hi=6.0, min_fraction=0.1)
        K = np.asarray(frame.K)
        t = c.arm.camera_pose(io.arm_command())
        out = dict(ok=True, u=u, v=v, depth=z, K=K.round(3).tolist(), T_base_cam=m4(t))
        if z is not None:
            xyz = np.array([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z])
            out.update(xyz_cam=xyz.round(4).tolist(), xyz_base=(t @ np.append(xyz, 1.0))[:3].round(4).tolist())
        ray = t[:3, :3] @ np.array([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], 1.0])
        out["ray_base"] = ray.round(4).tolist()
        out["origin_base"] = t[:3, 3].round(4).tolist()
        if ray[2] < -0.05:
            s = -t[2, 3] / ray[2]
            out["floor_hit"] = (t[:3, 3] + s * ray).round(3).tolist()
        return out

    def nav(self):
        nav = self.c.exp.nav(create=False)
        if nav is None or not nav.running:
            return dict(running=False, waypoints={} if nav is None else nav.waypoints)
        pose = nav.pose_map()
        tf = None
        try:
            from rclpy.time import Time
            t = nav.tf_buffer.lookup_transform("map", "odom", Time()).transform
            from ..exp.navigation import quat_to_yaw
            tf = [t.translation.x, t.translation.y, quat_to_yaw(t.rotation)]
        except Exception:
            pass
        m = nav.map
        return dict(running=True, pose_map=pose, map_odom=tf, waypoints=nav.waypoints, goals=nav.goals[-30:],
                    frontiers=[list(map(float, p)) for p in nav.frontier_pts[:30]],
                    trace=[[round(t[1], 2), round(t[2], 2)] for t in nav.trace[-600::3]],
                    map=None if m is None else dict(res=m["res"], origin=m["origin"], size=list(m["grid"].shape),
                                                    known=int((m["grid"] >= 0).sum())))


def _light(metrics):
    """Metrics without heavy or non-JSON members."""
    out = {}
    for k, v in (metrics or {}).items():
        if k in ("video",):
            continue
        out[k] = v
    return out


def register(app, console):
    api = Live(console)
    console.live = api
    import pathlib
    app.mount("/static", StaticFiles(directory=str(pathlib.Path(__file__).parent / "static")), name="static")

    @app.get("/api/exp/live")
    def live(since: float = 0.0):
        return api.live(since)

    @app.post("/api/exp/fk")
    def fk(body: dict = Body(...)):
        return api.fk(body["servo"])

    @app.post("/api/exp/ik")
    def ik(body: dict = Body(...)):
        return api.ik(body["target"], body.get("pitch", 70), body.get("seed"))

    @app.get("/api/exp/reach")
    def reach(pitch: int = 70, y: float = 0.0):
        return api.reach(pitch, y)

    @app.get("/api/exp/pick")
    def pick(u: float, v: float):
        return api.pick(u, v)

    @app.get("/api/exp/nav")
    def nav():
        return api.nav()
