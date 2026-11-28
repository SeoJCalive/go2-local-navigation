"""G03 coordinate/measurement contracts; pure data, no simulation or ROS launch."""

import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).parent / "acceptance"))
from saved_map import GOALS, WORLD_MAP, amcl_errors, map_pose, physics_summary


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
