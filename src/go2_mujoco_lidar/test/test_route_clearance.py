"""Hand-derived distances test the route oracle independently of route values."""
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).parent / "acceptance"))
from route_clearance import clearance, obstacle_distance


def test_box_edge_corner_inside_and_rotated_distance():
    box = dict(type="box", pos="1 2 0", size="2 1 1")
    assert obstacle_distance((4, 2), box) == 1
    assert obstacle_distance((6, 7), box) == 5  # corner offset (3,4)
    assert obstacle_distance((1, 2), box) == -1
    box["euler"] = f"0 0 {math.pi/2}"
    assert obstacle_distance((1, 5), box) == pytest.approx(1)


def test_cylinder_surface_distance():
    cylinder = dict(type="cylinder", pos="0 0 1", size="2 1")
    assert obstacle_distance((3, 4), cylinder) == 3


def test_continuous_segment_bound_and_collision_negative_control(tmp_path):
    world = tmp_path / "world.xml"
    world.write_text('<mujoco><worldbody><geom name="island" type="box" '
                     'pos="2 1 1" size=".5 .5 1"/></worldbody></mujoco>')
    clear = clearance(world, [[4, 0]], radius=.4)
    assert clear["minimum"] == pytest.approx(.095)  # .5 edge gap - .4 - .005
    blocked = clearance(world, [[4, .8]], radius=.4)
    assert blocked["minimum"] < 0
