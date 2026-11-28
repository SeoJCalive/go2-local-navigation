"""G04 preregistered route and offline verdict contracts; no simulation."""
from collections import Counter
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parent / "acceptance"))
from narrow_passage import (GOAL_SPECS, THROATS, SafetyReader, crossings, entered,
                            goal_placement, negative_outcome, passage_coverage, recoveries, verdict_physics)
from saved_map import map_pose
from safety_stop import stationary, post_stop_summary


def test_route_counts_transform_and_boundary_last():
    assert Counter(s["kind"] for s in GOAL_SPECS) == {
        "positive": 34, "negative": 4, "boundary": 4}
    assert len({s["id"] for s in GOAL_SPECS}) == 42
    assert [s["id"] for s in GOAL_SPECS[-4:]] == ["B01", "B02", "B03", "B04"]
    assert [s["passage"] for s in GOAL_SPECS if s["kind"] == "negative"] == [
        "P07", "P08", "P09", "P10"]
    for spec in GOAL_SPECS:
        assert spec["map_goal"] == map_pose(*spec["world_goal"])
        assert spec["time_limit_s"] == 150


def test_crossing_requires_opposite_sides_and_finite_interior():
    throat = ((0, -1), (0, 1))
    assert crossings([(-1, 0), (1, 0)], throat) == [.5]
    assert crossings([(-1, 0), (0, 0), (1, 0)], throat) == [1]
    for points in ([(-1, 0), (0, 0)], [(-1, 1), (1, 1)],
                   [(-1, 2), (1, 2)], [(-1, 0), (-.1, 0)]):
        assert not crossings(points, throat)
    assert entered([(.15, 0)], throat)
    assert not entered([(.150001, 0)], throat)


def test_negative_needs_aborted_deadline_no_path_no_entry_and_blocked_costmap():
    row = {"status": 6, "wall_sec": 150, "planned_path_count": 0,
           "tolerance_allowed_cells": 0, "passage": "P07"}
    assert all(negative_outcome(row, [(0, 0)]).values())
    for key, value in (("status", 4), ("status", "timeout"), ("wall_sec", 150.001),
                       ("planned_path_count", 1), ("tolerance_allowed_cells", None),
                       ("tolerance_allowed_cells", 1)):
        assert not all(negative_outcome({**row, key: value}, [(0, 0)]).values())
    assert not negative_outcome(row, [(-3.2, -2.27)])["no_entry"]
    assert not negative_outcome(row, [(-3.4, -2.27), (-3, -2.27)])["no_entry"]
    assert not negative_outcome(row, [])["no_entry"]


def test_boundary_frames_excluded_from_main_verdict_but_retained_as_raw():
    rows = [{"sim_time": 1, "min_clearance_m": .2, "fall": 0},
            {"sim_time": 2, "min_clearance_m": .01, "fall": 1}]
    main = verdict_physics(rows, 2)
    assert main["frames"] == 1 and main["min_clearance_m"] == .2 and main["falls"] == 0
    assert verdict_physics(rows, float("inf"))["falls"] == 1
    assert len(rows) == 2


def test_coverage_needs_independent_gt_and_real_positive_paths():
    assert not passage_coverage([], [])["passed"]
    truth = [{"x": -3.5, "y": 0}, {"x": -3, "y": 0}]
    path = {"kind": "positive", "poses": [map_pose(-3.5, 0), map_pose(-3, 0)]}
    result = passage_coverage(truth, [path])
    assert result["gt"]["P14"] and result["planned"]["P14"]
    assert not result["passed"]  # other 15 throats missing
    assert not passage_coverage(truth, [{**path, "kind": "boundary"}])["planned"]["P14"]
    # Two disconnected path messages must never create a crossing between them.
    assert not passage_coverage(truth, [
        {"kind": "positive", "poses": [map_pose(-3.5, 0)]},
        {"kind": "positive", "poses": [map_pose(-3, 0)]}])["planned"]["P14"]


def test_recovery_terminal_and_actual_behavior_sender():
    log = ("ACTION_STATUS\t1000000000\t/spin/_action/status\tuuid:2\n"
           "ACTION_STATUS\t2000000000\t/spin/_action/status\tuuid:4\n"
           "ACTION_STATUS\t3000000000\t/wait/_action/status\tlater:2\n")
    ownership = {"messages": [
        {"received_steady_ns": 1500000000, "node": "/behavior_server", "publisher_gid": "real"},
        {"received_steady_ns": 1600000000, "node": "/controller_server", "publisher_gid": "other"}]}
    rows = recoveries(log, ownership)
    assert rows[0]["wall_sec"] == 1
    assert rows[0]["terminal"] == 4
    assert rows[0]["senders"] == [("/behavior_server", "real")]
    assert rows[1]["terminal"] is None


def test_safety_reader_consumes_only_new_complete_frames(tmp_path):
    physics = tmp_path / "physics.csv"
    physics.write_text("sim_time,min_clearance_m,fall\n1,.2,0\n")
    (tmp_path / "contacts.csv").write_text("sim_time,robot_geom\n")
    reader = SafetyReader(tmp_path)
    assert reader.poll() is None
    assert reader.minimum == .2
    with physics.open("a") as file:
        file.write("2,.09")
    assert reader.poll() is None
    with physics.open("a") as file:
        file.write(",0\n")
    assert "clearance=0.09" in reader.poll()
    assert reader.poll() is None
    with (tmp_path / "contacts.csv").open("a") as file:
        file.write("3,17\n")
    assert "접촉" in reader.poll()


def test_safety_reader_retains_first_alert_frame(tmp_path):
    (tmp_path / "physics.csv").write_text(
        "sim_time,min_clearance_m,fall\n1,.2,0\n2,.09,0\n3,.02,0\n")
    reader = SafetyReader(tmp_path)
    assert "sim=2" in reader.poll()
    assert reader.alert_row["sim_time"] == "2"
    assert reader.minimum == .02


def test_free_goal_placement_checks_actual_cost_and_inscribed_region_margin():
    grid = {"width": 400, "height": 400, "origin": [-10, -10], "resolution": .05,
            "data": [0]*160000}
    grid["data"][0] = 99
    assert goal_placement(grid)["passed"]
    spec = next(s for s in GOAL_SPECS if s["id"] == "N04")
    x, y = spec["map_goal"][:2]
    i, j = int((x+10)/.05), int((y+10)/.05)
    grid["data"][j*400+i] = 1
    assert not goal_placement(grid)["passed"]
    grid["data"][j*400+i] = 0
    grid["data"][j*400+i+1] = 99
    assert not goal_placement(grid)["passed"]
    negatives = [s for s in GOAL_SPECS if s["kind"] == "negative"]
    assert [s["world_goal"][:2] for s in negatives] == [
        [-3.2, -2.27], [-6.2, 2.65], [-6.2, 3.85], [5.9, 2.08]]


def test_gt_stationary_requires_linear_and_angular_speed():
    rows = [dict(sim_time=i*.05, x=0., y=0., qw=1., qx=0., qy=0., qz=0.)
            for i in range(30)]
    assert stationary(rows)
    assert not stationary([])
    assert not stationary([{**r, "x": r["sim_time"]*.031} for r in rows])
    import math
    assert not stationary([{**r, "qw": math.cos(r["sim_time"]*.11/2),
                            "qz": math.sin(r["sim_time"]*.11/2)} for r in rows])


def test_post_alert_uses_all_physics_frames_and_both_bounds():
    start = {"sim_time": 2., "x": 0., "y": 0.}
    physics = [{"sim_time": 1, "x": 0, "y": 0, "min_clearance_m": .001},
               {"sim_time": 2, "x": 0, "y": 0, "min_clearance_m": .10},
               {"sim_time": 3, "x": .09, "y": 0, "min_clearance_m": .06}]
    result = post_stop_summary(physics, start, True)
    assert result["passed"] and result["frames"] == 2
    assert result["max_displacement_m"] == .09
    assert result["min_clearance_m"] == .06
    assert not post_stop_summary(physics, start, False)["passed"]
    assert not post_stop_summary([], start, True)["passed"]
    physics[-1]["x"] = .101
    assert not post_stop_summary(physics, start, True)["passed"]
    physics[-1]["x"], physics[-1]["min_clearance_m"] = .09, .049
    assert not post_stop_summary(physics, start, True)["passed"]
    # Motion after the next goal resumes is outside the completed stop episode.
    assert post_stop_summary(physics, start, True, 2)["passed"]
