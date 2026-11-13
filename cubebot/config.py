"""Mission configuration: defaults here, overrides from a JSON file."""
import copy
import json
from pathlib import Path

from .exp.settings import EXP_DEFAULTS

DEFAULTS = {
    "cube": {
        # Starting guess only: the mission measures the real edge length from depth and replaces it.
        "size_m": 0.035,
        "size_min_m": 0.02,
        "size_max_m": 0.08,
        "hsv_lo": [18, 100, 110],
        "hsv_hi": [36, 255, 255],
        "min_pixels": 40,
        "size_tolerance": 0.45,
    },
    "box": {
        # "hsv" needs hsv_lo/hsv_hi; "geometry" looks for a low open container by depth.
        "mode": "geometry",
        "hsv_lo": [5, 60, 40],
        "hsv_hi": [20, 255, 255],
        "min_width_m": 0.12,
        "max_width_m": 0.45,
        "min_height_m": 0.04,
        "max_height_m": 0.22,
        # The arm reaches ~0.26 m ahead of the base at floor level and the chassis ends at 0.125 m (0.145 with the lidars),
        # so the cube has to be dropped within a few cm of the near rim to be pulled out again.
        "freeze_within_m": 0.45,
        "near_edge_m": 0.165,
        "drop_x_m": 0.22,
        "drop_max_m": 0.245,                      # must stay inside the arm floor-reach window or the cube cannot be taken out again
    },
    "base": {
        "max_vx": 0.10,
        "max_vy": 0.08,
        "max_wz": 0.40,
        "accel": 0.25,
        "angular_accel": 1.2,
        "control_hz": 20.0,
        "footprint": {"front": 0.145, "back": 0.16, "half_width": 0.15},
        "stop_margin_m": 0.12,
        "slow_margin_m": 0.35,
        "dock_stop_margin_m": 0.05,
        "dock_slow_margin_m": 0.12,
    },
    "manual_base": {"max_vx": 0.30, "max_vy": 0.25, "max_wz": 0.80, "accel": 0.45, "angular_accel": 1.5},
    "arm": {
        "search_pose": [90, 130, 0, 25, 90, 30],
        "near_search_pose": [90, 130, 0, 5, 90, 30],   # camera 45 deg down: sees the floor from ~0.25 m
        "carry_pose": [90, 130, 0, 25, 90, 95],
        "safe_pose": [90, 90, 90, 90, 90, 30],
        "gripper_open": 30,
        "gripper_close": 125,                      # mechanical end: fingers touch when empty
        "gripper_width_open_m": 0.11,              # jaw gap at gripper_open; linear in the angle down to 0 at gripper_close
        "gripper_overdrive_deg": 55,               # command this far past the estimated contact angle: the clamp force
        "gripper_hold_max": 165,                   # never command more than this while holding
        "gripper_close_step": 5,
        "gripper_feedback_valid": True,           # measured: stalls ~13 deg short of the command on a cube, reaches it when empty
        "tcp_offset_m": 0.085,
        "grasp_bias_base_m": (-0.02, 0.0, 0.0),   # target shift in the base frame; the sign is chosen below by test
        "move_ms_min": 800,
        "move_ms_per_deg": 25,
        "max_step_deg": 10,
        "grasp_pitch_deg": 70.0,
        "pregrasp_height_m": 0.08,
        "lift_height_m": 0.12,
        "floor_margin_m": 0.012,
    },
    "reach": {
        "stand_off_m": 0.19,
        "xy_tolerance_m": 0.012,
        "yaw_tolerance_rad": 0.06,
    },
    "camera": {
        "optical_rotation": None,
        "min_depth_m": 0.20,
        "max_depth_m": 3.0,
        "floor_inlier_m": 0.015,
    },
    "odometry": {
        "gyro_weight": 0.97,
        "gyro_scale": 1.0,
        "lidar_correction": False,              # live ICP made the yaw jump by tens of degrees; wheels + gyro only
        "icp_max_correction_m": 0.06,
        "icp_max_correction_rad": 0.09,
        "icp_min_inlier_ratio": 0.55,
        "icp_max_rms_m": 0.035,
        "lidar_self_mask": {"x": [-0.19, 0.19], "y": [-0.17, 0.17]},
        "lidar_mounts": {
            "scan0": [-0.11617, 0.09156, 0.0],
            "scan1": [0.10766, -0.09078, 0.0],
        },
    },
    "mission": {
        "search_turn_deg": 30,
        "search_settle_s": 0.7,
        "search_relocate_m": 0.45,
        "max_search_rounds": 1,
        "grasp_retries": 3,
        "place_retries": 2,
        "extract_retries": 3,
        "min_battery_v": 10.9,
        "max_runtime_s": 900,
        "final_action": "place_on_floor",
        "stop_file": "/tmp/cubebot.stop",
    },
    "exp": EXP_DEFAULTS,
}


def merge(base, override):
    out = copy.deepcopy(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = merge(out[key], value)
        else:
            out[key] = value
    return out


def load(path=None):
    override = json.loads(Path(path).read_text()) if path else {}
    return merge(DEFAULTS, override)
