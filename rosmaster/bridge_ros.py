"""ROS 2 wrapper: presents the native-protocol controller exactly like the stock `robotio` micro-ROS node.

Publishes /odom_raw, /imu/data_raw, /imu/mag, /battery, /scan0, /scan1, /arm6_feedback (+ /arm_servo_state with NaN for
invalid readings). Subscribes /cmd_vel, /arm6_joints, /arm_joint, /beep. Run: python3 -m rosmaster.bridge_ros
"""
import json
import math
import sys
from pathlib import Path

import rclpy
from arm_msgs.msg import ArmJoint, ArmJoints
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, LaserScan, MagneticField
from std_msgs.msg import Float32, UInt16

from .core import BridgeCore
from .link import Link

CAL_PATHS = [Path(__file__).resolve().parent.parent / "config" / "controller-calibration.json",
             Path.home() / "Explorer/config/controller-calibration.json"]


class RosBridge(Node):
    def __init__(self, port="/dev/ttyUSB0"):
        super().__init__("robotio")
        cal_path = next(p for p in CAL_PATHS if p.exists())
        calibration = json.loads(cal_path.read_text())["joints"]
        self.link = Link(port)
        self.core = BridgeCore(self.link, calibration, self.publish, log=lambda m: self.get_logger().info(m))
        self.link.callbacks.append(self.core.on_frame)
        self.pubs = dict(
            odom=self.create_publisher(Odometry, "/odom_raw", qos_profile_sensor_data),
            imu=self.create_publisher(Imu, "/imu/data_raw", qos_profile_sensor_data),
            mag=self.create_publisher(MagneticField, "/imu/mag", qos_profile_sensor_data),
            battery=self.create_publisher(Float32, "/battery", qos_profile_sensor_data),
            scan0=self.create_publisher(LaserScan, "/scan0", qos_profile_sensor_data),
            scan1=self.create_publisher(LaserScan, "/scan1", qos_profile_sensor_data),
            arm_servo=self.create_publisher(JointState, "/arm_servo_state", qos_profile_sensor_data),
            arm_fb=self.create_publisher(ArmJoints, "/arm6_feedback", qos_profile_sensor_data))
        self.create_subscription(Twist, "/cmd_vel", lambda m: self.core.on_cmd_vel(m.linear.x, m.linear.y, m.angular.z),
                                 QoSProfile(depth=1))
        self.create_subscription(ArmJoints, "/arm6_joints", self.on_arm6, 1)
        self.create_subscription(ArmJoint, "/arm_joint", self.on_arm1, 1)
        self.create_subscription(UInt16, "/beep", lambda m: self.core.on_beep(m.data > 0), 1)
        self.create_timer(0.02, self.core.tick)
        self.last_arm = [90.0] * 6
        self._last_fb = 0.0

    def on_arm6(self, m):
        self.last_arm = [float(m.joint1), float(m.joint2), float(m.joint3), float(m.joint4), float(m.joint5), float(m.joint6)]
        self.core.on_arm(self.last_arm, m.time)

    def on_arm1(self, m):
        if 1 <= m.id <= 6:
            self.last_arm[m.id - 1] = float(m.joint)
            self.core.on_arm(self.last_arm, m.time)

    def publish(self, topic, data):
        now = self.get_clock().now().to_msg()
        if topic == "battery":
            self.pubs["battery"].publish(Float32(data=float(data)))
        elif topic == "odom":
            x, y, yaw = data["pose"]
            vx, vy, wz = data["velocity"]
            m = Odometry()
            m.header.stamp, m.header.frame_id, m.child_frame_id = now, "odom_raw", "base_footprint"
            m.pose.pose.position.x, m.pose.pose.position.y = x, y
            m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z = vx, vy, wz
            for i in (0, 7, 35):
                m.pose.covariance[i], m.twist.covariance[i] = 0.05, 0.02
            self.pubs["odom"].publish(m)
        elif topic == "imu":
            m = Imu()
            m.header.stamp, m.header.frame_id = now, "imu_frame"
            m.orientation_covariance[0] = -1.0
            m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = data["accel"]
            m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = data["gyro"]
            for i in (0, 4, 8):
                m.angular_velocity_covariance[i], m.linear_acceleration_covariance[i] = 0.01, 0.1
            self.pubs["imu"].publish(m)
        elif topic == "mag":
            m = MagneticField()
            m.header.stamp, m.header.frame_id = now, "imu_frame"
            m.magnetic_field.x, m.magnetic_field.y, m.magnetic_field.z = data
            self.pubs["mag"].publish(m)
        elif topic in ("scan0", "scan1"):
            m = LaserScan()
            m.header.stamp, m.header.frame_id = now, "laser%s_frame" % topic[-1]
            m.angle_min, m.angle_increment = 0.0, 2 * math.pi / 666
            m.angle_max = 665 * m.angle_increment
            m.scan_time = (data["ended_us"] - data["acquired_us"]) / 1e6
            m.time_increment = m.scan_time / 666
            m.range_min, m.range_max = 0.02, 12.0
            m.ranges, m.intensities = list(data["ranges"]), list(data["intensities"])
            self.pubs[topic].publish(m)
        elif topic == "arm_servo":
            m = JointState()
            m.header.stamp = now
            m.name = ["servo%d" % i for i in range(1, 7)]
            m.position = [float("nan") if v is None else float(v) for v in data]
            self.pubs["arm_servo"].publish(m)
            if all(v is not None for v in data) and self.core.clock() - self._last_fb > 0.05:
                self._last_fb = self.core.clock()
                fb = ArmJoints()
                (fb.joint1, fb.joint2, fb.joint3, fb.joint4, fb.joint5, fb.joint6) = [int(round(v)) for v in data]
                self.pubs["arm_fb"].publish(fb)


def main():
    rclpy.init()
    node = RosBridge(sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.link.estop()
        except Exception:
            pass
        node.link.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
