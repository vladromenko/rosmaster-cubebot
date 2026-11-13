"""Defaults of the five experiments (merged into cfg["exp"]; override with --config JSON like the rest)."""

EXP_DEFAULTS = {
    "paths": {"results": "~/CubeBot/results", "data": "~/CubeBot/data", "maps": "~/CubeBot/maps",
              "models": "~/CubeBot/models"},
    # Camera looks forward and slightly up from 0.44 m: a standing person is visible from 0.6 m to ~5 m.
    "people_pose": [90, 90, 90, 10, 90, 30],
    "detector": {"person_score": 0.40, "object_score": 0.25, "low_score": 0.20, "threads": 4, "people_every": 1},
    "tracker": {"high_score": 0.40, "max_age_s": 1.2, "iou_gate": 0.05, "accept_cost": 0.78, "hist_weight": 0.45,
                "reid_max_dist": 0.50, "min_hits_to_lock": 3},
    "follow": {
        "standoff_m": 1.2, "deadband_m": 0.12, "min_dist_m": 0.70, "max_dist_m": 4.5,
        "kp_x": 0.65, "kp_y": 0.55, "kp_wz": 1.5,
        "max_vx": 0.40, "max_back": 0.12, "max_vy": 0.25, "max_wz": 0.80, "accel": 0.5, "angular_accel": 2.0,
        "ff_gain": 0.9,                 # share of the target's own velocity added to the command
        "stale_s": 0.7,                 # no fresh measurement of the target for this long -> no motion at all
        "search_s": 12.0,               # look for a lost target at most this long, then give up
        "search_wz": 0.35,
        "lock_max_dist_m": 3.0, "lock_center_px": 220,
        "avoid_clear_m": 0.55,          # steer around lidar obstacles closer than this along the path
    },
    "gesture": {
        "window_s": 1.6, "hold_s": 0.7, "stop_motion_max": 0.12, "release_s": 0.6, "cooldown_s": 1.5,
        "curl_hi": 1.65, "curl_lo": 1.40, "beckon_cycles": 2, "push_growth": 1.35, "push_window_s": 0.8,
        "point_min_ratio": 1.4, "point_mirror": False,
        "turn_deg": 45.0, "step_m": 0.4, "come_stop_m": 0.9, "speed": 0.15, "turn_speed": 0.6,
        "trial_window_s": 7.0, "hand_crop": True,
    },
    "nav": {
        "map_resolution": 0.05, "max_laser_range": 5.0, "scan_bins": 720,
        "max_vx": 0.18, "max_vy": 0.12, "max_wz": 0.55,
        "footprint": [[0.145, 0.15], [0.145, -0.15], [-0.16, -0.15], [-0.16, 0.15]],
        "inflation_m": 0.10, "goal_tol_xy": 0.10, "goal_tol_yaw": 0.2,
        "frontier_min_cells": 6, "frontier_clearance_m": 0.22, "explore_budget_s": 900, "goal_timeout_s": 120,
        "stuck_s": 25.0, "start_timeout_s": 90.0,
    },
    "retrieve": {
        "labels_ru": {"кубик": "cube", "куб": "cube", "бутылк": "bottle", "чашк": "cup", "кружк": "cup",
                      "мяч": "sports ball", "яблок": "apple", "банан": "banana", "мишк": "teddy bear",
                      "игрушк": "teddy bear", "апельсин": "orange"},
        "max_object_width_m": 0.10, "approach_standoff_m": 0.22, "deliver_standoff_m": 0.0,
        "grasp_retries": 3, "search_rounds": 1, "map_search": True,
    },
    "assistant": {
        "person_standoff_m": 0.55, "hand_reach_x": [0.12, 0.30], "hand_reach_z": [0.05, 0.50],
        "handover_timeout_s": 25.0, "tug_deg": 4.0, "hand_near_m": 0.07, "person_search_turns": 12,
        "table_handover": False,
    },
}
