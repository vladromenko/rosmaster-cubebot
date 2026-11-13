"""ROS 2 (Jazzy) binding for the Rosmaster M3 Pro, as exposed by the Explorer stack.

Topics (domain 30, micro-ROS node `robotio`): /cmd_vel, /arm6_joints, /odom_raw,
/imu/data_raw, /scan0, /scan1, /battery, /camera/{color,depth}/image_raw.
Camera topics need Explorer's SHM-enabled Fast DDS profile (see scripts/run.sh).
"""
import math
import threading
import time

import numpy as np
import rclpy
from arm_msgs.msg import ArmJoints
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
from std_msgs.msg import Bool, Float32

from .robot_io import BaseIO
from .scan_matching import build_scan, scan_to_base
from .vision import Frame


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class RosRobot(BaseIO):
    REALTIME = True
    CONTROL_HZ = 20.0

    def __init__(self, cfg, arm_model, log=print, node_name="cubebot", dry_run=False):
        super().__init__(cfg, arm_model, log)
        rclpy.init()
        self.dry_run = dry_run
        self.node = Node(node_name)
        n = self.node
        self.twist_pub = n.create_publisher(Twist, "/cmd_vel", QoSProfile(depth=1, lifespan=Duration(seconds=0.15)))
        self.arm_pub = n.create_publisher(ArmJoints, "/arm6_joints", 1)
        self.status_pub = n.create_publisher(Bool, "/cubebot/running", 1)
        n.create_subscription(Odometry, "/odom_raw", self._on_odom, qos_profile_sensor_data)
        n.create_subscription(Imu, "/imu/data_raw", self._on_imu, qos_profile_sensor_data)
        n.create_subscription(Float32, "/battery", self._on_battery, qos_profile_sensor_data)
        n.create_subscription(Bool, "/cubebot/stop", self._on_stop, 1)
        n.create_subscription(ArmJoints, "/arm6_feedback", self._on_arm_feedback, qos_profile_sensor_data)
        for name in ("scan0", "scan1"):
            n.create_subscription(LaserScan, "/" + name, lambda m, k=name: self._on_scan(k, m), qos_profile_sensor_data)
        n.create_subscription(Image, "/camera/color/image_raw", self._on_color, qos_profile_sensor_data)
        n.create_subscription(Image, "/camera/depth/image_raw", self._on_depth, qos_profile_sensor_data)
        n.create_subscription(CameraInfo, "/camera/color/camera_info", self._on_info, qos_profile_sensor_data)
        self._created = time.monotonic()
        self._lock = threading.Lock()
        self._scans = {}
        self._battery = None
        self._color = None
        self._depth = None
        self._K = None
        self._frame = None
        self._last_odom = 0.
        self.raw = dict(twist=(0., 0., 0.), wheel=(0., 0., 0.), gyro_wz=0.0, gyro_yaw=0.0)   # for the docs page
        self._last_imu = 0.
        self._last_scan_pair = 0.
        self._stop_cb = False
        self._running = True
        self.executor = MultiThreadedExecutor(num_threads=3)
        self.executor.add_node(n)
        threading.Thread(target=self.executor.spin, daemon=True).start()
        self._control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._control_thread.start()

    # -- time ---------------------------------------------------------------------
    def now(self):
        return time.monotonic()

    def _advance(self, dt):
        time.sleep(dt)

    def _control_loop(self):
        period = 1.0 / self.CONTROL_HZ
        last = time.monotonic()
        while self._running:
            time.sleep(max(0.0, period - (time.monotonic() - last)))
            now = time.monotonic()
            dt, last = now - last, now
            try:
                self.control_step(min(dt, 0.2))
            except Exception as exc:                       # never leave the base driving on an error
                self.log("control loop error: %r" % (exc,))
                self._publish_twist(0., 0., 0.)

    # -- actuators ------------------------------------------------------------------
    def _publish_twist(self, vx, vy, wz):
        msg = Twist()
        if not self.dry_run:
            msg.linear.x, msg.linear.y, msg.angular.z = float(vx), float(vy), float(wz)
        self.twist_pub.publish(msg)

    def _publish_arm(self, servo, duration_ms):
        if self.dry_run:
            self.log("dry-run: arm %s in %d ms" % (list(servo), duration_ms))
            return
        msg = ArmJoints()
        (msg.joint1, msg.joint2, msg.joint3, msg.joint4, msg.joint5, msg.joint6) = [int(v) for v in servo]
        msg.time = int(duration_ms)
        self.arm_pub.publish(msg)

    # -- sensors --------------------------------------------------------------------
    def battery(self):
        return self._battery

    def _on_arm_feedback(self, msg):
        self._arm_fb = ([float(msg.joint1), float(msg.joint2), float(msg.joint3), float(msg.joint4), float(msg.joint5),
                         float(msg.joint6)], time.monotonic())

    def measured_arm(self, max_age=1.0):
        fb = getattr(self, "_arm_fb", None)
        return fb[0] if fb and time.monotonic() - fb[1] < max_age else None

    def _on_battery(self, msg):
        self._battery = float(msg.data)

    def _on_stop(self, msg):
        # A stop published by a stale `stop.sh` can be delivered the moment a new node appears.
        if msg.data and time.monotonic() - self._created > 2.0:
            self.request_abort("stop requested on /cubebot/stop")

    def _on_odom(self, msg):
        now = time.monotonic()
        dt = min(now - self._last_odom, 0.2) if self._last_odom else 0.0
        self._last_odom = now
        tw = msg.twist.twist
        x, y, yaw = self.raw["wheel"]
        c, s = math.cos(yaw), math.sin(yaw)
        self.raw = dict(self.raw, twist=(tw.linear.x, tw.linear.y, tw.angular.z),
                        wheel=(x + (tw.linear.x * c - tw.linear.y * s) * dt, y + (tw.linear.x * s + tw.linear.y * c) * dt,
                               yaw + tw.angular.z * dt))
        p = msg.pose.pose
        if abs(p.position.x) + abs(p.position.y) > 1e-9 or abs(p.orientation.z) > 1e-9:
            self.odometry.on_wheel(p.position.x, p.position.y, _yaw(p.orientation))     # firmware integrates the pose itself
            return
        # The native firmware reports body velocities only: integrate them here.
        t = msg.twist.twist
        x, y, yaw = getattr(self, "_wheel_pose", (0.0, 0.0, 0.0))
        c, s = math.cos(yaw), math.sin(yaw)
        x += (t.linear.x * c - t.linear.y * s) * dt
        y += (t.linear.x * s + t.linear.y * c) * dt
        yaw += t.angular.z * dt
        self._wheel_pose = (x, y, yaw)
        self.odometry.on_wheel(x, y, yaw)

    def _on_imu(self, msg):
        self._last_imu = time.monotonic()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        wz = msg.angular_velocity.z
        last = getattr(self, "_raw_gyro_t", None)
        t = time.monotonic()
        if last is not None and 0 < t - last < 0.5:
            self.raw = dict(self.raw, gyro_wz=wz, gyro_yaw=self.raw["gyro_yaw"] + (wz - self.odometry._gyro_bias) * (t - last))
        else:
            self.raw = dict(self.raw, gyro_wz=wz)
        self._raw_gyro_t = t
        self.odometry.on_gyro(stamp if stamp > 0 else time.monotonic(), wz)

    def _on_scan(self, name, msg):
        mount = self.cfg["odometry"]["lidar_mounts"][name]
        pts, ok = scan_to_base(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max, mount)
        now = time.monotonic()
        with self._lock:
            self._scans[name] = (now, pts, ok)
            other = self._scans.get("scan1" if name == "scan0" else "scan0")
            if other is None or abs(other[0] - now) > 0.12 or now - self._last_scan_pair < 0.08:
                return
            self._last_scan_pair = now
            parts = [self._scans["scan0"][1:], self._scans["scan1"][1:]]
        scan = build_scan(parts, self.cfg["odometry"]["lidar_self_mask"])
        self.scan = scan
        self.odometry.on_scan(scan)

    def _on_info(self, msg):
        self._K = np.array(msg.k, float).reshape(3, 3)

    @staticmethod
    def _image(msg, channels):
        data = np.frombuffer(msg.data, np.uint8)
        return data.reshape(msg.height, msg.width, channels) if channels else data

    def _on_color(self, msg):
        if msg.encoding not in ("rgb8", "bgr8"):
            return
        rgb = self._image(msg, 3)
        if msg.encoding == "bgr8":
            rgb = rgb[..., ::-1]
        self._color = (time.monotonic(), np.ascontiguousarray(rgb))
        self._pair()

    def _on_depth(self, msg):
        if msg.encoding != "16UC1":
            return
        depth = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width).astype(np.float32) * 0.001
        self._depth = (time.monotonic(), depth)
        self._pair()

    def _pair(self):
        color, depth, K = self._color, self._depth, self._K
        if color is None or depth is None or K is None:
            return
        if abs(color[0] - depth[0]) > 0.1:
            return
        # Depth is registered to the colour frame, so one intrinsic matrix serves both.
        self._frame = Frame(color[1], depth[1], K, max(color[0], depth[0]))

    def _latest_frame(self):
        return self._frame

    def sensors_alive(self, max_age=1.0):
        now = time.monotonic()
        return dict(odom=now - self._last_odom < max_age, imu=now - self._last_imu < max_age,
                    scan=self.scan is not None and now - self._last_scan_pair < max_age,
                    camera=self._frame is not None and now - self._frame.stamp < max_age,
                    battery=self._battery is not None)

    def shutdown(self):
        self._running = False
        self._control_thread.join(timeout=1.0)
        super().shutdown()
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown()
