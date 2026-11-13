"""Kinematic simulator: room, yellow cube, open box, two lidars, RGB-D camera, noisy odometry.

Not physics: it exists to exercise the whole mission loop (perception, odometry,
IK, grasp logic) with deliberate error sources — wheel slip, gyro bias, a constant
offset between commanded and true servo angles, depth noise.
"""
import math

import numpy as np

from .geometry import rot_z, transform
from .robot_io import BaseIO
from .scan_matching import build_scan
from .vision import Frame

FLOOR_RGB = (120, 120, 125)
WALL_RGB = (215, 215, 210)
CUBE_RGB = (240, 205, 20)
BOX_RGB = (150, 95, 60)


def _ray_aabb(origin, dirs, lo, hi):
    """Nearest positive hit distance of rays with an axis-aligned box (inf if none)."""
    with np.errstate(divide="ignore", invalid="ignore"):
        inv = 1.0 / dirs
        t1 = (lo - origin) * inv
        t2 = (hi - origin) * inv
    tmin = np.nanmax(np.minimum(t1, t2), axis=1)
    tmax = np.nanmin(np.maximum(t1, t2), axis=1)
    hit = (tmax >= np.maximum(tmin, 0.0)) & np.isfinite(tmin)
    t = np.where(tmin > 0, tmin, tmax)
    return np.where(hit, t, np.inf)


class World:
    def __init__(self, room=(4.0, 3.0), cube=(1.6, 0.2), box=(1.2, -0.9, 0.0), box_size=(0.24, 0.18, 0.10),
                 cube_size=0.035, box_clutter=None):
        self.room = room
        self.cube_size = cube_size
        self.cube = np.array([cube[0], cube[1], cube_size / 2])
        self.cube_held = False
        self.box_center = np.array(box[:2])
        self.box_yaw = box[2]
        self.box_size = box_size            # (length x, width y, height) in the box frame
        self.wall = 0.012
        self.floor = 0.01

    def box_slabs(self):
        """Box walls and floor as axis-aligned slabs in the box frame."""
        L, W, H = self.box_size
        t = self.wall
        return [((-L / 2, -W / 2, 0), (L / 2, W / 2, self.floor)),
                ((-L / 2, -W / 2, 0), (L / 2, -W / 2 + t, H)),
                ((-L / 2, W / 2 - t, 0), (L / 2, W / 2, H)),
                ((-L / 2, -W / 2, 0), (-L / 2 + t, W / 2, H)),
                ((L / 2 - t, -W / 2, 0), (L / 2, W / 2, H))]

    def inside_box_xy(self, xy, margin=0.0):
        c, s = math.cos(-self.box_yaw), math.sin(-self.box_yaw)
        d = xy - self.box_center
        local = np.array([c * d[0] - s * d[1], s * d[0] + c * d[1]])
        L, W, _ = self.box_size
        return abs(local[0]) < L / 2 - self.wall - margin and abs(local[1]) < W / 2 - self.wall - margin

    def lidar(self, pose, mount, angles, max_range=12.0):
        """Ranges for rays from a lidar mounted on a robot at `pose` (x, y, yaw)."""
        origin = np.array(pose[:2]) + rot_z(pose[2]) @ np.array(mount[:2])
        yaw = pose[2] + mount[2]
        d = np.stack([np.cos(angles + yaw), np.sin(angles + yaw)], axis=1)
        ranges = np.full(len(angles), np.inf)
        hx, hy = self.room[0] / 2, self.room[1] / 2
        for axis, bound in ((0, hx), (0, -hx), (1, hy), (1, -hy)):
            with np.errstate(divide="ignore", invalid="ignore"):
                t = (bound - origin[axis]) / d[:, axis]
            t = np.where(t > 0, t, np.inf)
            ranges = np.minimum(ranges, t)
        # The box is visible to a lidar at 12.5 cm only if its walls are taller than that.
        if self.box_size[2] > 0.125:
            for lo, hi in self.box_slabs()[1:]:
                local = rot_z(-self.box_yaw) @ (origin - self.box_center)
                ld = d @ rot_z(-self.box_yaw).T
                t = _ray_aabb(np.append(local, 0.125), np.column_stack([ld, np.zeros(len(ld))]),
                              np.array(lo), np.array(hi))
                ranges = np.minimum(ranges, t)
        ranges[ranges > max_range] = np.inf
        return ranges


class SimRobot(BaseIO):
    SCAN_DT = 1 / 7.0
    ODOM_DT = 0.1
    IMU_DT = 1 / 25.0
    IMG_SCALE = 2          # render at 320x240 to keep ray casting cheap

    def __init__(self, cfg, arm_model, world, seed=1, log=print, error_scale=1.0):
        super().__init__(cfg, arm_model, log)
        self.world = world
        self.rng = np.random.default_rng(seed)
        self.t = 0.0
        self.true_pose = np.array([0.0, 0.0, 0.0])
        self.true_servo = np.array(cfg["arm"]["safe_pose"], float)
        self._servo_from = self.true_servo.copy()
        self._servo_to = self.true_servo.copy()
        self._servo_t0 = 0.0
        self._servo_dur = 0.0
        self.servo_offset = self.rng.uniform(-1.5, 1.5, 6) * error_scale
        self.servo_offset[5] = 0.0
        self.slip = np.array([0.94, 0.88, 1.07]) if error_scale else np.ones(3)
        self.gyro_bias = 0.012 * error_scale
        self._wheel_pose = np.array([0.0, 0.0, 0.0])
        self._next_scan = self._next_odom = self._next_imu = 0.0
        self._scan_parts = {}
        self.K_full = np.array([[480.7, 0, 322.3], [0, 480.7, 245.6], [0, 0, 1.0]])
        self.frame_cache = None
        self.path_length = 0.0
        self._collided = False
        self.events = []
        self.odometry.on_wheel(*self._wheel_pose)

    # -- BaseIO hooks ------------------------------------------------------------
    def now(self):
        return self.t

    def battery(self):
        return 12.0

    def _publish_twist(self, vx, vy, wz):
        self._twist = (vx, vy, wz)

    def measured_arm(self, max_age=1.0):
        servo = self._current_servo() - self.servo_offset
        if self.world.cube_held:
            servo[5] = min(servo[5], 72.0)              # the fingers stop on the cube
        return [float(v) for v in servo]

    def _publish_arm(self, servo, duration_ms):
        self._servo_from = self._current_servo()
        self._servo_to = np.array(servo, float) + self.servo_offset
        self._servo_t0 = self.t
        self._servo_dur = duration_ms / 1000.0
        self.events.append(("arm", self.t, list(servo)))

    _twist = (0., 0., 0.)

    def _current_servo(self):
        if self._servo_dur <= 0:
            return self._servo_to.copy()
        a = min(1.0, (self.t - self._servo_t0) / self._servo_dur)
        a = a * a * (3 - 2 * a)
        return self._servo_from + (self._servo_to - self._servo_from) * a

    def _advance(self, dt):
        self.control_step(dt)
        self.t += dt
        vx, vy, wz = self._twist
        true = np.array([vx * self.slip[0], vy * self.slip[1], wz * self.slip[2]])
        yaw = self.true_pose[2]
        step = np.array([math.cos(yaw) * true[0] - math.sin(yaw) * true[1],
                         math.sin(yaw) * true[0] + math.cos(yaw) * true[1], true[2]]) * dt
        # Hard collision with the room walls: the sim robot simply cannot pass.
        new = self.true_pose + step
        hx, hy = self.world.room[0] / 2 - 0.2, self.world.room[1] / 2 - 0.2
        if abs(new[0]) > hx or abs(new[1]) > hy:
            self._collided = True
            new[:2] = self.true_pose[:2]
        self.path_length += float(np.linalg.norm(new[:2] - self.true_pose[:2]))
        self.true_pose = new
        # Wheel odometry reports the commanded motion without the slip the floor imposes.
        cmd = np.array([vx, vy, wz])
        moved = np.linalg.norm(step[:2]) > 0 or abs(step[2]) > 0
        if moved:
            wy = self._wheel_pose[2]
            self._wheel_pose += np.array([math.cos(wy) * cmd[0] - math.sin(wy) * cmd[1],
                                          math.sin(wy) * cmd[0] + math.cos(wy) * cmd[1], cmd[2]]) * dt
            self._wheel_pose[:2] += self.rng.normal(0, 0.0004, 2)
        self._sense()
        self._update_held_cube()

    def _sense(self):
        if self.t >= self._next_odom:
            self._next_odom += self.ODOM_DT
            self.odometry.on_wheel(*self._wheel_pose)
        if self.t >= self._next_imu:
            self._next_imu += self.IMU_DT
            wz = self._twist_true_wz() + self.gyro_bias + self.rng.normal(0, 0.004)
            self.odometry.on_gyro(self.t, wz)
        if self.t >= self._next_scan:
            self._next_scan += self.SCAN_DT
            parts = []
            for name, (lo, hi) in (("scan0", (0.0, 1.5 * math.pi)), ("scan1", (-math.pi, 0.5 * math.pi))):
                mount = self.cfg["odometry"]["lidar_mounts"][name]
                angles = np.linspace(lo, hi, 500, endpoint=False)
                r = self.world.lidar(self.true_pose, mount, angles) + self.rng.normal(0, 0.004, 500)
                inc = (hi - lo) / 500
                from .scan_matching import scan_to_base
                pts, ok = scan_to_base(r, lo, inc, 0.05, 12.0, mount)
                parts.append((pts, ok))
            self.scan = build_scan(parts, self.cfg["odometry"]["lidar_self_mask"])
            self.odometry.on_scan(self.scan)

    def _twist_true_wz(self):
        return self._twist[2] * self.slip[2]

    # -- held cube ---------------------------------------------------------------
    def tcp_world(self):
        t_base = transform((*self.true_pose[:2], 0.0), np.block(
            [[rot_z(self.true_pose[2]), np.zeros((2, 1))], [np.zeros((1, 2)), np.ones((1, 1))]]))
        tcp = self.arm.fk(self._current_servo())["tcp"][:3, 3]
        return (t_base @ np.append(tcp, 1.0))[:3]

    def _update_held_cube(self):
        gripper = self._current_servo()[5] - self.servo_offset[5]
        w = self.world
        if w.cube_held:
            if gripper < 60:           # released: drops straight down
                w.cube_held = False
                pos = self.tcp_world()
                xy = pos[:2]
                z = w.cube_size / 2 + (w.floor if w.inside_box_xy(xy, 0.01) else 0.0)
                w.cube = np.array([xy[0], xy[1], z])
                self.events.append(("release", self.t, w.cube.copy(), bool(w.inside_box_xy(xy))))
            else:
                w.cube = self.tcp_world()
        elif gripper > 75 and np.linalg.norm(self.tcp_world() - w.cube) < 0.022:
            w.cube_held = True
            self.events.append(("grasp", self.t, w.cube.copy()))

    # -- camera ------------------------------------------------------------------
    def _latest_frame(self):
        if self.frame_cache is not None and self.frame_cache.stamp >= self.t - 1e-6:
            return self.frame_cache
        self.frame_cache = self.render()
        return self.frame_cache

    def render(self):
        s = self.IMG_SCALE
        w, h = 640 // s, 480 // s
        K = self.K_full.copy()
        K[:2] /= s
        K[2, 2] = 1.0
        vs, us = np.mgrid[0:h, 0:w]
        dirs_cam = np.stack([(us - K[0, 2]) / K[0, 0], (vs - K[1, 2]) / K[1, 1], np.ones_like(us, float)], axis=-1)
        dirs_cam = dirs_cam.reshape(-1, 3)
        servo = self._current_servo()
        t_base_cam = self.arm.fk(servo)["camera"]
        c, sn = math.cos(self.true_pose[2]), math.sin(self.true_pose[2])
        t_world_base = np.array([[c, -sn, 0, self.true_pose[0]], [sn, c, 0, self.true_pose[1]], [0, 0, 1, 0], [0, 0, 0, 1.]])
        t_world_cam = t_world_base @ t_base_cam
        origin = t_world_cam[:3, 3]
        dirs = dirs_cam @ t_world_cam[:3, :3].T            # ray directions (not unit: z=1 in camera)
        depth = np.full(len(dirs), np.inf)
        color = np.zeros((len(dirs), 3), np.uint8)

        def take(t, rgb):
            nonlocal depth, color
            closer = t < depth
            depth = np.where(closer, t, depth)
            color[closer] = rgb

        with np.errstate(divide="ignore", invalid="ignore"):
            tf = -origin[2] / dirs[:, 2]
        take(np.where(tf > 0, tf, np.inf), FLOOR_RGB)
        hx, hy = self.world.room[0] / 2, self.world.room[1] / 2
        for axis, bound in ((0, hx), (0, -hx), (1, hy), (1, -hy)):
            with np.errstate(divide="ignore", invalid="ignore"):
                tw = (bound - origin[axis]) / dirs[:, axis]
            hit_z = origin[2] + tw * dirs[:, 2]
            take(np.where((tw > 0) & (hit_z < 2.5), tw, np.inf), WALL_RGB)
        half = self.world.cube_size / 2
        take(_ray_aabb(origin, dirs, self.world.cube - half, self.world.cube + half), CUBE_RGB)
        rb = rot_z(-self.world.box_yaw)
        local_o = np.append(rb @ (origin[:2] - self.world.box_center), origin[2])
        local_d = np.column_stack([dirs[:, :2] @ rb.T, dirs[:, 2]])
        for lo, hi in self.world.box_slabs():
            take(_ray_aabb(local_o, local_d, np.array(lo), np.array(hi)), BOX_RGB)
        depth = depth.reshape(h, w)
        color = color.reshape(h, w, 3)
        # Registered depth: values near the sensor floor and beyond range read as invalid.
        d = depth + self.rng.normal(0, 0.0015 * depth ** 2, depth.shape)
        d[(depth < 0.22) | (depth > 4.0)] = 0.0
        return Frame(color, d.astype(np.float32), K, self.t)

    # -- ground truth helpers ------------------------------------------------------
    def cube_in_box(self):
        return bool(self.world.inside_box_xy(self.world.cube[:2]) and not self.world.cube_held)

    def cube_outside_and_free(self):
        return bool(not self.world.cube_held and not self.world.inside_box_xy(self.world.cube[:2], -0.03))
