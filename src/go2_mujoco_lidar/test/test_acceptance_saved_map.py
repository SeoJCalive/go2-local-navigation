"""G03 coordinate/measurement contracts; pure data, no simulation or ROS launch."""

import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).parent / "acceptance"))
from saved_map import (COMMAND_TOPICS, GOALS, WORLD_MAP, amcl_errors,
                       command_ownership, map_pose, physics_summary)


def test_h3_transform_and_goal_order():
    a, tx, ty = WORLD_MAP
    assert GOALS == [[.8, 0.], [1.5, -1.], [0., 0.]]
    assert map_pose(0, 0) == [tx, ty, a]
    x, y, angle = map_pose(1.5, -1., .3)
    assert math.cos(a)*(x-tx)+math.sin(a)*(y-ty) == pytest.approx(1.5)
    assert -math.sin(a)*(x-tx)+math.cos(a)*(y-ty) == pytest.approx(-1.)
    assert angle == pytest.approx(.3+a)


def test_amcl_uses_equal_stamp_interpolated_gt_in_map():
    truth = [{"sim_time": "10", "x": "0", "y": "0"},
             {"sim_time": "12", "x": "2", "y": "-2"}]
    x, y, angle = map_pose(1, -1)
    rows = amcl_errors([[9, x, y, angle], [11, x+.3, y+.4, angle],
                        [13, x, y, angle]], truth)
    assert [r["xy_error_m"] for r in rows] == [None, pytest.approx(.5), None]
    assert amcl_errors([[11, x, y, angle]], truth)[0]["xy_error_m"] == pytest.approx(0.)
    assert amcl_errors([[11, x, y, angle]], []) [0]["xy_error_m"] is None


def test_physics_minimum_keeps_location_and_inclusive_goal_interval():
    rows = [{"sim_time": t, "min_clearance_m": gap, "fall": fall,
             "x": x, "y": 0, "robot_geom": 7, "world_geom": 3}
            for t, gap, fall, x in [(1, .2, 0, 0), (2, .1, 0, .8),
                                     (3, -.01, 1, 1.5)]]
    result = physics_summary(rows, 1, 2)
    assert result["frames"] == 2
    assert result["min_clearance_m"] == .1
    assert result["minimum"]["x"] == .8
    assert result["falls"] == 0
    assert physics_summary(rows)["min_clearance_m"] == -.01
    assert physics_summary(rows)["falls"] == 1
    assert physics_summary(rows, 4, 5) == {
        "frames": 0, "min_clearance_m": None, "falls": None, "minimum": None}


def command_fixture(second_ns, second_handle="h2", second_node="/behavior_server"):
    topic = COMMAND_TOPICS[0]
    samples = [{"cmd_vel": [
        {"topic": topic, "gid": "g1", "node": "/controller_server"},
        {"topic": topic, "gid": "g2", "node": second_node},
        {"topic": topic, "gid": "g3", "node": "/behavior_server"},
        {"topic": topic, "gid": "g4", "node": "/behavior_server"},
    ]}]
    log = ("COMMAND_OWNER\t1000000000\t10\t" + topic + "\th1\t0\t0\t0\t0\t0\t0\n"
           f"COMMAND_OWNER\t{second_ns}\t11\t{topic}\t{second_handle}\t0\t0\t0\t0\t0\t0\n"
           "GID_MAP\th1\tg1\nGID_MAP\th2\tg2\n")
    return log, samples


def test_v2_accepts_idle_endpoints_and_exact_watchdog_boundary():
    log, samples = command_fixture(1250000000)
    result = command_ownership(log, samples, 10, 11)
    assert result["passed"]
    assert result["endpoint_counts"] == [4]
    assert [m["node"] for m in result["messages"]] == ["/controller_server", "/behavior_server"]
    assert not result["overlaps"]


def test_v2_rejects_overlap_even_for_zero_commands_or_same_behavior_node():
    log, samples = command_fixture(1249999999)
    assert not command_ownership(log, samples, 10, 11)["passed"]
    samples[0]["cmd_vel"][0]["node"] = "/behavior_server"
    assert len(command_ownership(log, samples, 10, 11)["overlaps"]) == 1
    log, samples = command_fixture(1000000001, "h1")
    assert command_ownership(log, samples, 10, 11)["passed"]  # same GID can stream


@pytest.mark.parametrize("handle,node", [
    ("unmapped", "/behavior_server"), ("h2", "/teleop")])
def test_v2_fails_unknown_sender_or_external_endpoint(handle, node):
    log, samples = command_fixture(2000000000, handle, node)
    assert not command_ownership(log, samples, 10, 11)["passed"]


def test_v2_needs_observations_and_uses_sim_time_only_for_phase_selection():
    log, samples = command_fixture(1250000000)
    assert not command_ownership(log, samples, 20, 30)["passed"]
    assert not command_ownership(log, [], 10, 11)["passed"]
    assert not command_ownership("", samples, 10, 11)["passed"]


def test_v2_also_checks_bypass_command_topic():
    log, samples = command_fixture(1250000000, "h2", "/teleop")
    log = log.replace(COMMAND_TOPICS[0], COMMAND_TOPICS[1])
    for endpoint in samples[0]["cmd_vel"]:
        endpoint["topic"] = COMMAND_TOPICS[1]
    result = command_ownership(log, samples, 10, 11)
    assert not result["passed"]
    assert result["external_endpoints"][0]["topic"] == "/cmd_vel"


def test_v3_action_channel_correlates_intervals_without_overriding_unknown_gid():
    log, samples = command_fixture(1250000000)
    log += ("ACTION_STATUS\t900000000\t/follow_path/_action/status\tgoal1:1\n"
            "ACTION_STATUS\t950000000\t/follow_path/_action/status\tgoal1:2\n"
            "ACTION_STATUS\t1100000000\t/follow_path/_action/status\tgoal1:4\n"
            "ACTION_STATUS\t1200000000\t/spin/_action/status\tgoal2:2\n"
            "ACTION_STATUS\t1300000000\t/spin/_action/status\tgoal2:3\n"
            "ACTION_STATUS\t1400000000\t/spin/_action/status\tgoal2:5\n")
    result = command_ownership(log, samples, 10, 11)
    assert result["passed"]
    assert [row["active_actions"] for row in result["messages"]] == [
        ["/follow_path/_action/status"], ["/spin/_action/status"]]
    assert result["action_status"]["intervals"] == [
        {"topic": "/follow_path/_action/status", "goal_id": "goal1",
         "start_ns": 900000000, "end_ns": 1100000000},
        {"topic": "/spin/_action/status", "goal_id": "goal2",
         "start_ns": 1200000000, "end_ns": 1400000000}]
    unknown = command_ownership(log.replace("\th1\t", "\tunmapped\t", 1), samples, 10, 11)
    assert not unknown["passed"]
    assert unknown["fallback_messages"] == 0
    assert unknown["attribution_counts"]["UNIDENTIFIED"] == 1


def test_v3_no_mapping_cap_and_empty_status_closes_interval():
    topic = COMMAND_TOPICS[0]
    endpoints, lines = [], []
    for i in range(300):
        endpoints.append({"topic": topic, "gid": f"g{i}", "node": "/controller_server"})
        lines.extend([f"GID_MAP\th{i}\tg{i}",
                      f"COMMAND_OWNER\t{1000000000+i*250000000}\t10\t{topic}"
                      f"\th{i}\t0\t0\t0\t0\t0\t0"])
    lines += ["ACTION_STATUS\t900000000\t/follow_path/_action/status\tgoal:2",
              "ACTION_STATUS\t1000000000\t/follow_path/_action/status\t"]
    result = command_ownership("\n".join(lines), [{"cmd_vel": endpoints}], 10, 10)
    assert result["passed"]
    assert result["attribution_counts"] == {"/controller_server": 300}
    assert len(result["gid_attribution_counts"]) == 300
    assert result["action_status"]["messages_with_active_action"] == 0


def test_v2_safety_gid_allows_only_zero_after_stop_request():
    log, samples = command_fixture(1250000000, "h2", "/go2_sim_acceptance")
    stop = {"request_steady_ns": 1200000000,
            "zero_endpoints": [{"gid": "g2", "node": "/go2_sim_acceptance"}]}
    assert not command_ownership(log, samples, 10, 11)["passed"]
    result = command_ownership(log, samples, 10, 11, stop)
    assert result["passed"] and result["messages"][1]["safety_zero"]
    assert command_ownership(log, samples, 10, 11, [stop])["passed"]
    assert not command_ownership(log, samples, 10, 11, {
        **stop, "request_steady_ns": 1250000001})["passed"]
    assert not command_ownership(log.replace("h2\t0\t0", "h2\t.1\t0"),
                                 samples, 10, 11, stop)["passed"]
    assert not command_ownership(log, samples, 10, 11, {
        **stop, "zero_endpoints": [{"gid": "g2", "node": "/other"}]})["passed"]
