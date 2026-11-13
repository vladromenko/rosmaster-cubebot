"""Run the full mission in the simulator: python -m scripts.sim_run [--seed N]."""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from cubebot import config
from cubebot.arm_model import ArmModel
from cubebot.mission import Mission
from cubebot.sim import SimRobot, World


def build(seed=1, cube=(1.1, 0.35), box=(0.9, -0.8, 0.4), box_size=(0.24, 0.18, 0.10), error_scale=1.0, cfg=None,
          cube_size=None):
    cfg = cfg or config.load()
    cfg["arm"]["gripper_feedback_valid"] = True       # the simulated gripper reports its true angle
    cfg["odometry"]["lidar_correction"] = True
    cfg["arm"]["grasp_bias_base_m"] = (0.0, 0.0, 0.0)  # the real-robot calibration bias does not exist in the sim
    arm = ArmModel(cfg["arm"]["tcp_offset_m"])
    world = World(cube=cube, box=box, box_size=box_size, cube_size=cube_size or cfg["cube"]["size_m"])
    return cfg, SimRobot(cfg, arm, world, seed=seed, log=lambda *_: None, error_scale=error_scale), world


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument("--truth", action="store_true")
    args = ap.parse_args()
    cfg, sim, world = build(args.seed)
    log = (lambda *_: None) if args.quiet else (lambda m: print("t=%6.1f %s" % (sim.t, m)))
    sim.log = log
    t0 = time.time()
    mission = Mission(sim, cfg, log)
    original_close = mission.close_gripper

    def traced_close():
        c, sn = np.cos(sim.true_pose[2]), np.sin(sim.true_pose[2])
        tw = np.array([[c, -sn, 0, sim.true_pose[0]], [sn, c, 0, sim.true_pose[1]], [0, 0, 1, 0], [0, 0, 0, 1.]])
        cam = tw @ sim.arm.fk(sim._current_servo())["camera"]
        cube_cam = (np.linalg.inv(cam) @ np.append(world.cube, 1))[:3]
        tcp_cam = sim.arm.tcp_in_camera(sim._current_servo())[:3]
        log("closing gripper: true TCP-cube distance %.1f mm; cube in camera %s, TCP in camera %s, model tcp_cam %s" % (
            1e3 * np.linalg.norm(sim.tcp_world() - world.cube), np.round(cube_cam, 3), np.round(tcp_cam, 3),
            np.round(mission.arm.tcp_in_camera(sim.arm_command())[:3], 3)))
        original_close()
    mission.close_gripper = traced_close
    if "--truth" in sys.argv:
        original_look = mission.look

        def traced_look(*a, **k):
            look = original_look(*a, **k)
            c, sn = np.cos(sim.true_pose[2]), np.sin(sim.true_pose[2])
            tw = np.array([[c, -sn, 0, sim.true_pose[0]], [sn, c, 0, sim.true_pose[1]], [0, 0, 1, 0], [0, 0, 0, 1.]])
            cam = tw @ sim.arm.fk(sim._current_servo())["camera"]
            log("   truth: cube in camera %s" % np.round((np.linalg.inv(cam) @ np.append(world.cube, 1))[:3], 3))
            return look
        mission.look = traced_look
    ok = mission.run()
    print("OK" if ok else "FAILED", "sim time %.0fs wall %.0fs" % (sim.t, time.time() - t0))
    print("cube in box:", sim.cube_in_box(), "cube out & free:", sim.cube_outside_and_free(), "path %.2f m" % sim.path_length)
    print("odometry error: fused %s true %s" % (np.round(sim.odometry.current(), 3), np.round(sim.true_pose, 3)))
    print("odometry stats", sim.odometry.stats)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
