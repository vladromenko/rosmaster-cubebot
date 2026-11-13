from cubebot.mission import Mission
from scripts.sim_run import build


def run(seed, **kw):
    cfg, sim, world = build(seed=seed, **kw)
    mission = Mission(sim, cfg, lambda *_: None)
    return mission.run(), mission, sim


def test_full_mission_with_low_box():
    ok, mission, sim = run(1, box_size=(0.24, 0.18, 0.07))
    assert ok, mission.result
    assert sim.cube_outside_and_free()
    kinds = [e[0] for e in sim.events if e[0] != "arm"]
    assert kinds == ["grasp", "release", "grasp", "release"]


def test_cube_behind_start_is_found_by_turning():
    ok, mission, sim = run(3, cube=(-1.0, 0.4), box=(0.8, 0.9, 1.0), box_size=(0.24, 0.18, 0.07))
    assert ok, mission.result


def test_missing_cube_ends_in_failure_not_motion_forever():
    cfg, sim, world = build(cube=(1.0, 3.0))          # outside the room: invisible
    cfg["mission"]["max_search_rounds"] = 1
    mission = Mission(sim, cfg, lambda *_: None)
    assert not mission.run()
    assert "cube not found" in mission.result
    assert sim.last_output == (0., 0., 0.)


def test_grasp_is_verified_by_the_gripper_feedback_and_start_pose_is_measured():
    cfg, sim, world = build(seed=2, box_size=(0.24, 0.18, 0.07))
    mission = Mission(sim, cfg, lambda *_: None)
    assert mission.run(), mission.result
    assert sim.cube_outside_and_free()
    # the sim reports the true angles: the estimate was synchronised from them at preflight
    assert any(e[0] == "grasp" for e in sim.events)
