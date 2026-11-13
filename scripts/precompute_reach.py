"""Precompute floor-reach windows into config/reach_cache.json: python -m scripts.precompute_reach [cube_size_m ...]"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cubebot import config
from cubebot.arm_model import ArmModel

cfg = config.load()
arm = ArmModel(cfg["arm"]["tcp_offset_m"])
path = str(Path(__file__).resolve().parents[1] / "config" / "reach_cache.json")
for size in [float(a) for a in sys.argv[1:]] or [cfg["cube"]["size_m"]]:
    for z in (size / 2, size / 2 + 0.01):              # cube on the floor, cube on a 1 cm box bottom
        print("z=%.4f ->" % z, arm.reach_window(z, cfg["arm"]["search_pose"], cache_files=(path,)))
