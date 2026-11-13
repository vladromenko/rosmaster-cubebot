"""Experiment 3: mapping, exploration and navigation with slam_toolbox + Nav2 (stock ROS 2 Jazzy packages).

What is ours: the glue. The two lidars are merged into one virtual /scan_merged at base_link, the fused wheel+gyro
odometry is published as /odom and the odom->base_link TF, and Nav2's velocity output (/nav_cmd_vel) is fed to
BaseIO.drive(), i.e. through the same deadman / lidar guard / acceleration limits as every other motion.
SLAM (slam_toolbox, online async) provides /map and the map->odom TF; Nav2 (planner, DWB omni controller, behaviours)
does the driving. Exploration is frontier based: free cells next to unknown cells become Nav2 goals.
"""
import json
import math
import os
import signal
import subprocess
import threading
import time

import cv2
import numpy as np
import yaml
from scipy import ndimage

from ..geometry import wrap
from .common import Experiment, base_limits, expand, put_text

NAV2_DEFAULT_PARAMS = "/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml"
SLAM_DEFAULT_PARAMS = "/opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml"


class _NoAlias(yaml.SafeDumper):
    """ROS 2's parameter parser does not support YAML anchors/aliases."""

    def ignore_aliases(self, data):
        return True


def dump_yaml(data, path):
    with open(path, "w") as fh:
        yaml.dump(data, fh, Dumper=_NoAlias, default_flow_style=False)


def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


def quat_to_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


# ----------------------------------------------------------------------------------------------- parameters
def build_nav2_params(ncfg):
    """Stock nav2_params.yaml reduced to the servers we need, with an omni DWB controller and our scan topic."""
    with open(NAV2_DEFAULT_PARAMS) as fh:
        full = yaml.safe_load(fh)
    keep = ("bt_navigator", "controller_server", "local_costmap", "global_costmap", "planner_server", "behavior_server")
    p = {k: full[k] for k in keep}
    p["bt_navigator"]["ros__parameters"]["odom_topic"] = "/odom"
    # The Jetson is busy (camera, lidar, SLAM, Python console): TF can be a few hundred ms late. Measured on this robot: with the
    # default 0.1-0.3 s the controller aborted goals with "Transform data too old when converting from odom to map".
    p["bt_navigator"]["ros__parameters"]["transform_tolerance"] = 1.0
    c = p["controller_server"]["ros__parameters"]
    c["controller_frequency"] = 20.0
    c["failure_tolerance"] = 1.0
    c["progress_checker"]["required_movement_radius"] = 0.15
    c["progress_checker"]["movement_time_allowance"] = 12.0
    c["general_goal_checker"].update(xy_goal_tolerance=ncfg["goal_tol_xy"], yaw_goal_tolerance=ncfg["goal_tol_yaw"])
    c["FollowPath"] = {
        "plugin": "dwb_core::DWBLocalPlanner", "debug_trajectory_details": False,
        "min_vel_x": -0.08, "min_vel_y": -ncfg["max_vy"], "max_vel_x": ncfg["max_vx"], "max_vel_y": ncfg["max_vy"],
        "max_vel_theta": ncfg["max_wz"], "min_speed_xy": 0.0, "max_speed_xy": ncfg["max_vx"], "min_speed_theta": 0.0,
        "acc_lim_x": 0.5, "acc_lim_y": 0.5, "acc_lim_theta": 1.5, "decel_lim_x": -0.6, "decel_lim_y": -0.6,
        "decel_lim_theta": -1.5, "vx_samples": 10, "vy_samples": 8, "vtheta_samples": 16, "sim_time": 1.6,
        "linear_granularity": 0.05, "angular_granularity": 0.05, "transform_tolerance": 1.0, "xy_goal_tolerance": 0.1,
        "trans_stopped_velocity": 0.05, "short_circuit_trajectory_evaluation": True, "stateful": True,
        "critics": ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"],
        "BaseObstacle.scale": 0.05, "PathAlign.scale": 32.0, "PathAlign.forward_point_distance": 0.1,
        "GoalAlign.scale": 24.0, "GoalAlign.forward_point_distance": 0.1, "PathDist.scale": 32.0, "GoalDist.scale": 24.0,
        "RotateToGoal.scale": 32.0, "RotateToGoal.slowing_factor": 5.0, "RotateToGoal.lookahead_time": -1.0,
    }
    scan_src = {"topic": "/scan_merged", "max_obstacle_height": 2.0, "clearing": True, "marking": True,
                "data_type": "LaserScan", "raytrace_max_range": 4.0, "raytrace_min_range": 0.0,
                "obstacle_max_range": 3.5, "obstacle_min_range": 0.0}
    obstacle = {"plugin": "nav2_costmap_2d::ObstacleLayer", "enabled": True, "observation_sources": "scan", "scan": scan_src}
    inflation = {"plugin": "nav2_costmap_2d::InflationLayer", "cost_scaling_factor": 6.0,
                 "inflation_radius": ncfg["inflation_m"] + 0.2}
    lc = p["local_costmap"]["local_costmap"]["ros__parameters"]
    for k in ("robot_radius", "voxel_layer", "static_layer"):
        lc.pop(k, None)
    lc.update(plugins=["obstacle_layer", "inflation_layer"], obstacle_layer=obstacle, inflation_layer=inflation,
              footprint=str(ncfg["footprint"]), update_frequency=8.0, publish_frequency=2.0, width=3, height=3,
              transform_tolerance=1.0,
              resolution=ncfg["map_resolution"])
    gc = p["global_costmap"]["global_costmap"]["ros__parameters"]
    gc.pop("robot_radius", None)
    gc.update(footprint=str(ncfg["footprint"]), obstacle_layer=obstacle, inflation_layer=inflation,
              resolution=ncfg["map_resolution"], update_frequency=2.0, transform_tolerance=1.0)
    p["planner_server"]["ros__parameters"]["expected_planner_frequency"] = 1.0
    pl = p["planner_server"]["ros__parameters"]["GridBased"]
    pl.update(plugin="nav2_navfn_planner::NavfnPlanner", tolerance=0.35, use_astar=True, allow_unknown=True)
    b = p["behavior_server"]["ros__parameters"]
    b["max_rotational_vel"], b["min_rotational_vel"] = ncfg["max_wz"], 0.25
    return p


def build_slam_params(ncfg):
    with open(SLAM_DEFAULT_PARAMS) as fh:
        p = yaml.safe_load(fh)
    s = p["slam_toolbox"]["ros__parameters"]
    s.update(odom_frame="odom", map_frame="map", base_frame="base_link", scan_topic="/scan_merged", mode="mapping",
             resolution=ncfg["map_resolution"], max_laser_range=ncfg["max_laser_range"], minimum_travel_distance=0.12,
             minimum_travel_heading=0.25, map_update_interval=3.0, transform_publish_period=0.05, throttle_scans=2,
             use_scan_matching=True, use_scan_barycenter=True, do_loop_closing=True, minimum_time_interval=0.4,
             transform_timeout=0.3, tf_buffer_duration=30.0, debug_logging=False)
    return p


LAUNCH_NAV = '''import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params = os.environ["CUBEBOT_NAV2_PARAMS"]
    nodes = ["controller_server", "planner_server", "behavior_server", "bt_navigator"]
    out = "screen"
    return LaunchDescription([
        Node(package="nav2_controller", executable="controller_server", output=out, parameters=[params],
             remappings=[("cmd_vel", "nav_cmd_vel")]),
        Node(package="nav2_planner", executable="planner_server", name="planner_server", output=out, parameters=[params]),
        Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server", output=out,
             parameters=[params], remappings=[("cmd_vel", "nav_cmd_vel")]),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator", output=out, parameters=[params]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_navigation",
             output=out, parameters=[{"autostart": True}, {"node_names": nodes}, {"bond_timeout": 6.0}]),
    ])
'''


# ----------------------------------------------------------------------------------------------- frontiers
def find_frontiers(grid, resolution, origin_xy, robot_xy, cfg):
    """Free cells next to unknown space, clustered. Only clusters the robot can reach through free space are returned.

    grid: int8 HxW (-1 unknown, 0..100 occupancy). Returns a list of dicts sorted best-first."""
    free = grid == 0
    # Lidar rays leave one-cell holes between them: close those before calling anything "unknown".
    known = ndimage.binary_closing(grid != -1, structure=np.ones((3, 3)), iterations=2)
    unknown = ~known
    occ = grid >= 65
    clearance_cells = cfg["frontier_clearance_m"] / resolution
    dist = ndimage.distance_transform_edt(~occ)
    safe = free & (dist >= clearance_cells)
    unk_near = ndimage.binary_dilation(unknown, structure=ndimage.generate_binary_structure(2, 1))
    frontier = safe & unk_near
    reach_labels, _ = ndimage.label(safe, structure=np.ones((3, 3)))
    rc = (int((robot_xy[1] - origin_xy[1]) / resolution), int((robot_xy[0] - origin_xy[0]) / resolution))
    h, w = grid.shape
    if not (0 <= rc[0] < h and 0 <= rc[1] < w):
        return []
    robot_label = reach_labels[rc]
    if robot_label == 0:                                   # robot on a cell the clearance filter rejected: nearest safe cell
        ys, xs = np.nonzero(safe)
        if len(ys) == 0:
            return []
        i = int(np.argmin((ys - rc[0]) ** 2 + (xs - rc[1]) ** 2))
        robot_label = reach_labels[ys[i], xs[i]]
    labels, n = ndimage.label(frontier, structure=np.ones((3, 3)))
    out = []
    for i in range(1, n + 1):
        ys, xs = np.nonzero(labels == i)
        if len(ys) < cfg["frontier_min_cells"]:
            continue
        cy, cx = ys.mean(), xs.mean()
        k = int(np.argmin((ys - cy) ** 2 + (xs - cx) ** 2))
        y, x = ys[k], xs[k]
        if reach_labels[y, x] != robot_label:
            continue
        xy = np.array([origin_xy[0] + (x + 0.5) * resolution, origin_xy[1] + (y + 0.5) * resolution])
        d = float(np.linalg.norm(xy - np.asarray(robot_xy)))
        out.append(dict(xy=xy, size=int(len(ys)), dist=d, score=d - 0.04 * min(len(ys), 40) * resolution * 20))
    out.sort(key=lambda f: f["score"])
    return out


# ----------------------------------------------------------------------------------------------- the stack
class NavStack:
    """slam_toolbox + Nav2 as child processes, plus the in-process glue. Persists across experiment runs."""

    def __init__(self, io, cfg, log):
        self.io, self.cfg, self.log = io, cfg, log
        self.n = cfg["exp"]["nav"]
        self.node = io.node
        self.procs = []
        self.running = False
        self.drive_enabled = False
        self.map = None                       # dict(grid, res, origin, stamp)
        self.map_stamp = 0.0
        self.trace = []                       # (t, x, y, yaw) in the map frame
        self.plans = []
        self.waypoints = {}
        self.goals = []                       # frontier goals sent so far (for the picture)
        self.frontier_pts = []
        self._pub_run = False
        self._scan_seen = None
        self._lock = threading.Lock()
        self._load_waypoints()
        self._setup_ros()

    # ---- ROS plumbing
    def _setup_ros(self):
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import OccupancyGrid, Odometry, Path
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import LaserScan
        from tf2_ros import Buffer, TransformBroadcaster, TransformListener
        n = self.node
        reliable = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.scan_pub = n.create_publisher(LaserScan, "/scan_merged", reliable)
        self.odom_pub = n.create_publisher(Odometry, "/odom", reliable)
        self.tf_pub = TransformBroadcaster(n)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, n)
        n.create_subscription(OccupancyGrid, "/map", self._on_map, latched)
        n.create_subscription(Twist, "/nav_cmd_vel", self._on_cmd, 10)
        n.create_subscription(Path, "/plan", self._on_plan, 5)
        self.client = ActionClient(n, NavigateToPose, "navigate_to_pose")
        self._NavigateToPose = NavigateToPose
        self._LaserScan, self._Odometry = LaserScan, Odometry

    def _on_map(self, msg):
        grid = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        with self._lock:
            self.map = dict(grid=grid, res=msg.info.resolution, origin=(msg.info.origin.position.x, msg.info.origin.position.y))
            self.map_stamp = time.monotonic()

    def _on_plan(self, msg):
        pts = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])
        if len(pts):
            self.plans.append((time.monotonic(), pts))
            self.plans = self.plans[-40:]

    def _on_cmd(self, msg):
        if not self.drive_enabled:
            return
        vx, vy, wz = msg.linear.x, msg.linear.y, msg.angular.z
        # the wheels stall below ~0.03 m/s: a tiny command is either zero or a real step
        vx = 0.0 if abs(vx) < 0.01 else math.copysign(max(abs(vx), 0.035), vx)
        vy = 0.0 if abs(vy) < 0.01 else math.copysign(max(abs(vy), 0.035), vy)
        wz = 0.0 if abs(wz) < 0.04 else math.copysign(max(abs(wz), 0.12), wz)
        self.io.drive(vx, vy, wz, hold=0.35)

    def _publish_loop(self):
        from geometry_msgs.msg import TransformStamped
        n, io = self.node, self.io
        period = 1.0 / 30.0
        while self._pub_run:
            t0 = time.monotonic()
            stamp = n.get_clock().now().to_msg()
            x, y, yaw = io.pose()
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, "odom", "base_link"
            tf.transform.translation.x, tf.transform.translation.y = float(x), float(y)
            q = yaw_to_quat(yaw)
            tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = q
            self.tf_pub.sendTransform(tf)
            od = self._Odometry()
            od.header.stamp, od.header.frame_id, od.child_frame_id = stamp, "odom", "base_link"
            od.pose.pose.position.x, od.pose.pose.position.y = float(x), float(y)
            od.pose.pose.orientation.z, od.pose.pose.orientation.w = q[2], q[3]
            od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.angular.z = [float(v) for v in io.last_output]
            self.odom_pub.publish(od)
            scan = io.scan
            if scan is not None and scan is not self._scan_seen:
                self._scan_seen = scan
                self._publish_scan(scan, stamp)
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

    def _publish_scan(self, scan, stamp):
        pts = scan.points
        nb = self.n["scan_bins"]
        msg = self._LaserScan()
        msg.header.stamp, msg.header.frame_id = stamp, "base_link"
        msg.angle_min, msg.angle_max = -math.pi, math.pi - 2 * math.pi / nb
        msg.angle_increment = 2 * math.pi / nb
        msg.range_min, msg.range_max = 0.12, 12.0
        msg.scan_time, msg.time_increment = 0.17, 0.0
        ranges = np.full(nb, np.inf, np.float32)
        if len(pts):
            ang = np.arctan2(pts[:, 1], pts[:, 0])
            rng = np.hypot(pts[:, 0], pts[:, 1])
            idx = np.clip(((ang + math.pi) / msg.angle_increment).astype(int), 0, nb - 1)
            order = np.argsort(-rng)                               # write far first, so the nearest return wins
            ranges[idx[order]] = rng[order]
        ranges[ranges < msg.range_min] = np.inf
        msg.ranges = ranges.tolist()
        self.scan_pub.publish(msg)

    # ---- processes
    def start(self):
        if self.running:
            return
        d = expand(self.cfg["exp"]["paths"]["data"])
        nav_dir = os.path.join(d, "nav")
        os.makedirs(nav_dir, exist_ok=True)
        slam_file, nav_file, launch_file = [os.path.join(nav_dir, f) for f in ("slam.yaml", "nav2.yaml", "nav_launch.py")]
        dump_yaml(build_slam_params(self.n), slam_file)
        dump_yaml(build_nav2_params(self.n), nav_file)
        open(launch_file, "w").write(LAUNCH_NAV)
        logs = expand("~/CubeBot/logs")
        os.makedirs(logs, exist_ok=True)
        for path in (slam_file, nav_file):          # leftovers of a killed console would keep driving /nav_cmd_vel
            subprocess.run(["pkill", "-f", path], check=False)
        time.sleep(0.5)
        env = dict(os.environ, CUBEBOT_NAV2_PARAMS=nav_file)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        cmds = [("slam", "ros2 launch slam_toolbox online_async_launch.py slam_params_file:=%s use_sim_time:=false" % slam_file),
                ("nav2", "ros2 launch %s" % launch_file)]
        self._pub_run = True
        threading.Thread(target=self._publish_loop, daemon=True).start()
        for name, cmd in cmds:
            fh = open(os.path.join(logs, "nav_%s_%s.log" % (name, stamp)), "w")
            self.procs.append(subprocess.Popen(["bash", "-c", "source /opt/ros/jazzy/setup.bash; exec " + cmd], env=env,
                                               stdout=fh, stderr=subprocess.STDOUT, preexec_fn=os.setsid))
        self.running = True
        self.log("nav: slam_toolbox and Nav2 started (logs in %s)" % logs)

    def wait_ready(self, timeout):
        """Map published, map->base_link TF available and the NavigateToPose action server up."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.io.wait(0.5)
            if self.map is not None and self.pose_map() is not None and self.client.server_is_ready():
                return True
            dead = [p for p in self.procs if p.poll() is not None]
            if dead:
                raise RuntimeError("a navigation process exited early (see ~/CubeBot/logs/nav_*.log)")
        return False

    def stop(self):
        self.drive_enabled = False
        for p in self.procs:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
            except (ProcessLookupError, PermissionError):
                pass
        t0 = time.monotonic()
        while time.monotonic() - t0 < 6 and any(p.poll() is None for p in self.procs):
            time.sleep(0.2)
        for p in self.procs:
            if p.poll() is None:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
        self.procs = []
        self.running = False
        self._pub_run = False
        self.map = None
        self.log("nav: stopped")

    # ---- poses and waypoints
    def pose_map(self):
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_link", rclpy_time_zero())
        except Exception:
            return None
        t = tf.transform
        return (t.translation.x, t.translation.y, quat_to_yaw(t.rotation))

    def _wp_file(self):
        return os.path.join(expand(self.cfg["exp"]["paths"]["data"]), "waypoints.json")

    def _load_waypoints(self):
        try:
            self.waypoints = json.load(open(self._wp_file()))
        except (OSError, ValueError):
            self.waypoints = {}

    def save_waypoint(self, name):
        p = self.pose_map()
        if p is None:
            return None
        self.waypoints[name] = dict(x=round(p[0], 3), y=round(p[1], 3), yaw=round(p[2], 3), t=time.strftime("%H:%M:%S"))
        os.makedirs(os.path.dirname(self._wp_file()), exist_ok=True)
        json.dump(self.waypoints, open(self._wp_file(), "w"), indent=1)
        return self.waypoints[name]

    # ---- navigation
    def goto(self, x, y, yaw, timeout=None, label=""):
        """Send a NavigateToPose goal and block until it ends. Returns a result dict with measured errors."""
        io = self.io
        timeout = timeout or self.n["goal_timeout_s"]
        goal = self._NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        q = yaw_to_quat(yaw)
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = q[2], q[3]
        t0 = time.monotonic()
        plans0 = t0
        info = dict(label=label, goal=[round(x, 3), round(y, 3), round(yaw, 3)], status="unknown", recoveries=0)
        fb = {}
        send = self.client.send_goal_async(goal, feedback_callback=lambda m: fb.update(rec=m.feedback.number_of_recoveries,
                                                                                       left=m.feedback.distance_remaining))
        while not send.done():
            io.wait(0.05)
            if time.monotonic() - t0 > 10:
                info["status"] = "not accepted (timeout)"
                return self._finish(info, t0, plans0)
        handle = send.result()
        if not handle.accepted:
            info["status"] = "rejected"
            return self._finish(info, t0, plans0)
        result = handle.get_result_async()
        last_progress, best_left, last_trace = time.monotonic(), 1e9, 0.0
        try:
            while not result.done():
                io.wait(0.1)
                now = time.monotonic()
                p = self.pose_map()
                if p is not None and now - last_trace > 0.25:
                    self.trace.append((now, p[0], p[1], p[2]))
                    last_trace = now
                left = fb.get("left")
                if left is not None and left < best_left - 0.05:
                    best_left, last_progress = left, now
                if now - t0 > timeout:
                    info["status"] = "timeout"
                    break
                if now - last_progress > self.n["stuck_s"]:
                    info["status"] = "stuck (no progress %.0f s)" % self.n["stuck_s"]
                    break
        finally:
            if not result.done():
                handle.cancel_goal_async()
                io.stop_base()
        if result.done():
            status = result.result().status
            info["status"] = {4: "reached", 5: "cancelled", 6: "failed"}.get(status, "status %d" % status)
        info["recoveries"] = fb.get("rec", 0)
        return self._finish(info, t0, plans0)

    def _finish(self, info, t0, plans0):
        p = self.pose_map()
        g = info["goal"]
        if p is not None:
            info["final"] = [round(p[0], 3), round(p[1], 3), round(p[2], 3)]
            info["err_xy_m"] = round(math.hypot(p[0] - g[0], p[1] - g[1]), 3)
            info["err_yaw_deg"] = round(math.degrees(wrap(p[2] - g[2])), 1)
        info["seconds"] = round(time.monotonic() - t0, 1)
        info["replans"] = self._count_replans(plans0)
        info["ok"] = info["status"] == "reached"
        return info

    def _count_replans(self, start):
        """Number of times the global path changed materially (>0.12 m from the previous path) during a goal."""
        paths = [p for t, p in self.plans if t >= start]
        changes = 0
        for a, b in zip(paths, paths[1:]):
            m = min(len(a), len(b))
            if m < 3:
                continue
            d = np.min(np.linalg.norm(b[:min(len(b), 60), None, :] - a[None, :, :], axis=2), axis=1)
            if float(d.max()) > 0.12:
                changes += 1
        return changes

    # ---- map picture
    def save_map(self, name):
        d = expand(self.cfg["exp"]["paths"]["maps"])
        os.makedirs(d, exist_ok=True)
        base = os.path.join(d, name)
        r = subprocess.run(["bash", "-c", "source /opt/ros/jazzy/setup.bash; ros2 run nav2_map_server map_saver_cli -f %s "
                            "--ros-args -p save_map_timeout:=8.0" % base], capture_output=True, text=True, timeout=40)
        ok = os.path.exists(base + ".yaml")
        self.log("map saved: %s.yaml" % base if ok else "map save failed: %s" % r.stderr[-300:])
        return base + ".yaml" if ok else None

    def map_png(self, trace=True):
        with self._lock:
            m = self.map
        if m is None:
            img = np.full((120, 320, 3), 40, np.uint8)
            put_text(img, "no map yet", (10, 60))
            return cv2.imencode(".png", img)[1].tobytes()
        grid, res, (ox, oy) = m["grid"], m["res"], m["origin"]
        img = np.full(grid.shape + (3,), 127, np.uint8)
        img[grid == 0] = (255, 255, 255)
        img[grid >= 65] = (0, 0, 0)
        img = img[::-1]
        h = grid.shape[0]
        scale = max(1, int(600 / max(grid.shape)))
        img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)

        def px(x, y):
            return int((x - ox) / res * scale), int((h - 1 - (y - oy) / res) * scale)

        if trace and len(self.trace) > 1:
            pts = np.array([px(t[1], t[2]) for t in self.trace[-1500:]], np.int32)
            cv2.polylines(img, [pts], False, (255, 120, 0), 2)
        for name, w in self.waypoints.items():
            c = px(w["x"], w["y"])
            cv2.circle(img, c, 7, (0, 160, 0) if name != "home" else (200, 0, 200), 2)
            put_text(img, name, (c[0] + 8, c[1] - 6), (0, 120, 0), 0.5)
        for f in self.frontier_pts[:12]:
            cv2.circle(img, px(f[0], f[1]), 4, (0, 200, 255), -1)
        p = self.pose_map()
        if p is not None:
            c = px(p[0], p[1])
            cv2.circle(img, c, 7, (0, 0, 255), -1)
            tip = (int(c[0] + 14 * math.cos(p[2])), int(c[1] - 14 * math.sin(p[2])))
            cv2.line(img, c, tip, (0, 0, 255), 2)
        ys, xs = np.nonzero((np.asarray(img).sum(axis=2) != 127 * 3))
        if len(ys):
            pad = 20
            img = img[max(0, ys.min() - pad):ys.max() + pad, max(0, xs.min() - pad):xs.max() + pad]
        return cv2.imencode(".png", img)[1].tobytes()


def rclpy_time_zero():
    from rclpy.time import Time
    return Time()


# ----------------------------------------------------------------------------------------------- experiment
class NavExperiment(Experiment):
    key = "e3_nav"
    title = "3. Карта, исследование и навигация"

    def __init__(self, ctx, nav_getter):
        super().__init__(ctx)
        self.nav_getter = nav_getter           # manager.nav(create=True) -> NavStack
        self.nav = None
        self.log_rows = []
        self.tour_results = []

    def setup(self):
        self.nav = self.nav_getter()
        self.allow_runtime(float(self.params.get("runtime_s", 1500)))
        mode = self.params.get("mode", "explore")
        self.state = "running"
        if not self.nav.running:
            self.say("запуск slam_toolbox и Nav2 …")
            self.nav.start()
        if not self.nav.wait_ready(self.ecfg["nav"]["start_timeout_s"]):
            raise RuntimeError("SLAM/Nav2 не готовы за %.0f с (нет карты, TF или action-сервера)" % self.ecfg["nav"]["start_timeout_s"])
        self.nav.drive_enabled = True
        if "home" not in self.nav.waypoints or mode == "explore" and self.params.get("reset_home", True):
            w = self.nav.save_waypoint("home")
            self.say("исходная точка записана: %s" % w)
        self.say("готово: режим %s" % mode)

    def teardown(self):
        if self.nav is not None:
            self.nav.drive_enabled = False
        self.io.stop_base()

    def run(self):
        n = self.ecfg["nav"]
        mode = self.params.get("mode", "explore")
        with base_limits(self.io, max_vx=n["max_vx"], max_vy=n["max_vy"], max_wz=n["max_wz"], accel=0.5, angular_accel=1.5):
            if mode == "explore":
                self.explore()
            elif mode == "tour":
                self.tour(self.params.get("points") or [k for k in ("P1", "P2", "P3") if k in self.nav.waypoints], True)
            elif mode == "home":
                self.go_home()
            elif mode == "idle":
                while not self._stop.is_set():
                    self.sleep(0.5)

    # ---- exploration
    def explore(self):
        nav, n = self.nav, self.ecfg["nav"]
        budget = float(self.params.get("budget_s", n["explore_budget_s"]))
        t0 = time.monotonic()
        blacklist, empty, goals_done, failures = [], 0, 0, 0
        self.metrics.update(goals=0, failures=0, explored_cells=0)
        while time.monotonic() - t0 < budget and not self._stop.is_set():
            m, pose = nav.map, nav.pose_map()
            if m is None or pose is None:
                self.sleep(0.5)
                continue
            fs = find_frontiers(m["grid"], m["res"], m["origin"], pose[:2], n)
            fs = [f for f in fs if all(np.linalg.norm(f["xy"] - b) > 0.5 for b in blacklist)]
            nav.frontier_pts = [f["xy"] for f in fs]
            self.metrics["frontiers"] = len(fs)
            if not fs:
                empty += 1
                self.say("нет достижимых границ неизвестного (%d/3)" % empty)
                if empty >= 3:
                    break
                self.sleep(2.0)
                continue
            empty = 0
            f = fs[0]
            yaw = math.atan2(f["xy"][1] - pose[1], f["xy"][0] - pose[0])
            self.say("цель %d: граница %s, размер %d, до неё %.2f м" % (goals_done + 1, np.round(f["xy"], 2), f["size"], f["dist"]))
            nav.goals.append(f["xy"].tolist())
            res = nav.goto(f["xy"][0], f["xy"][1], yaw, label="frontier")
            goals_done += 1
            if not res["ok"]:
                failures += 1
                blacklist.append(f["xy"])
                self.say("цель не достигнута (%s): в чёрный список" % res["status"])
            self.metrics.update(goals=goals_done, failures=failures, explored_cells=int((m["grid"] >= 0).sum()),
                                last=res)
        known = int((nav.map["grid"] >= 0).sum()) if nav.map else 0
        area = known * self.ecfg["nav"]["map_resolution"] ** 2
        path_len = self._trace_length()
        stamp = time.strftime("%Y%m%d_%H%M%S")
        saved = nav.save_map("room_" + stamp)
        rec = dict(kind="explore", seconds=round(time.monotonic() - t0, 1), goals=goals_done, failures=failures,
                   known_area_m2=round(area, 2), path_m=round(path_len, 2), map=saved,
                   finished="frontiers exhausted" if empty >= 3 else "budget/stop", video=self._recorder.path)
        self.ctx.store.add(self.key, rec)
        self.result = "карта построена: %.1f м², %d целей (%d неудач), путь %.1f м" % (area, goals_done, failures, path_len)
        if goals_done and failures == goals_done:
            self.state = "failed"                  # not a single frontier goal was reached: the map is only what was seen from the start
            self.result = "исследование не удалось: ни одна цель не достигнута (%d). " % goals_done + self.result
        self.say(self.result)

    def _trace_length(self):
        t = np.array([(p[1], p[2]) for p in self.nav.trace])
        return float(np.linalg.norm(np.diff(t, axis=0), axis=1).sum()) if len(t) > 1 else 0.0

    # ---- waypoint tours / return home
    def tour(self, names, return_home):
        nav = self.nav
        order = list(names) + (["home"] if return_home and "home" in nav.waypoints else [])
        if not order:
            raise RuntimeError("нет сохранённых точек: сохраните P1, P2, P3")
        label = self.params.get("label", "tour")
        t0 = time.monotonic()
        results = []
        nav.trace.clear()
        for name in order:
            w = nav.waypoints.get(name)
            if w is None:
                results.append(dict(label=name, ok=False, status="no such waypoint"))
                continue
            self.say("едем в %s (%.2f, %.2f)" % (name, w["x"], w["y"]))
            r = nav.goto(w["x"], w["y"], w["yaw"], label=name)
            results.append(r)
            self.say("%s: %s, ошибка %.2f м / %.0f°, %d перепланирований, %d восстановлений" % (
                name, r["status"], r.get("err_xy_m", -1), r.get("err_yaw_deg", 0), r["replans"], r["recoveries"]))
            self.metrics["tour"] = results
        ok = all(r.get("ok") for r in results)
        rec = dict(kind="tour", label=label, points=order, ok=ok, seconds=round(time.monotonic() - t0, 1),
                   path_m=round(self._trace_length(), 2), results=results, new_obstacle=bool(self.params.get("new_obstacle")),
                   trajectory=[[round(t[1], 2), round(t[2], 2)] for t in nav.trace[::4]], video=self._recorder.path)
        self.ctx.store.add(self.key, rec)
        self.result = ("маршрут пройден без ручного управления" if ok else "маршрут не завершён") + ": " + ", ".join(
            "%s %s" % (r["label"], r["status"]) for r in results)

    def go_home(self):
        self.tour([], True)

    def command(self, name, **kw):
        if name == "waypoint":
            nm = kw.get("name", "P1")
            nav = self.nav or self.nav_getter(create=False)
            if nav is None or not nav.running:
                self.say("навигация не запущена: сначала запустите исследование")
                return False
            w = nav.save_waypoint(nm)
            self.say("точка %s сохранена: %s" % (nm, w))
        elif name == "save_map":
            if self.nav:
                self.nav.save_map("room_" + time.strftime("%Y%m%d_%H%M%S"))
        return True

    def overlay(self, img):
        m = self.metrics or {}
        put_text(img, "E3 %s goals=%s fail=%s frontiers=%s" % (self.params.get("mode"), m.get("goals"), m.get("failures"),
                                                              m.get("frontiers")), (8, 20), (0, 255, 255))
        return img
