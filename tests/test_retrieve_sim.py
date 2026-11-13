"""Experiment 4/5 pipeline on the cube-mission simulator: deliver-to-start and hold-for-handover modes."""
import numpy as np

from cubebot.exp.retrieve import Retriever
from scripts.sim_run import build


def run(deliver, seed=1, **kw):
    cfg, sim, world = build(seed=seed, box=(0.9, -0.8, 0.4), **kw)
    r = Retriever(sim, cfg, lambda *_: None, "cube", deliver, perception=None, nav=None)
    return r.run(), r, sim, world


def test_retrieve_cube_to_start_pose_is_confirmed_by_camera():
    ok, r, sim, world = run(dict(kind="start"))
    assert ok, r.result
    assert all(r.stages.values()), r.stages
    # the cube really lies on the floor near the spot in front of the start pose, outside the gripper
    assert sim.cube_outside_and_free()
    assert np.linalg.norm(np.asarray(world.cube[:2])) < 0.6


def test_hold_mode_keeps_the_cube_in_the_gripper_for_the_handover():
    ok, r, sim, world = run(dict(kind="hold"), seed=2)
    assert ok, r.result
    assert r.stages["grasped"] and not r.stages["delivered"]
    assert not r.cube_released
    assert sim.arm_command()[5] > 100              # gripper still closed on the cube after the run (no parking drop)


def test_missing_object_fails_cleanly_with_reason():
    cfg, sim, world = build(cube=(1.0, 3.0))
    cfg["mission"]["max_search_rounds"] = 1
    r = Retriever(sim, cfg, lambda *_: None, "cube", dict(kind="start"), perception=None, nav=None)
    assert not r.run()
    assert "not found" in (r.fail_reason or "")
    assert sim.last_output == (0., 0., 0.)
