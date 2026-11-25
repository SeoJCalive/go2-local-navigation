"""Domain 65 Nav2 설정·launch·failure BT의 비물리 계약을 검증한다."""

from pathlib import Path
from typing import Final
from xml.etree import ElementTree

import yaml
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from launch_test_support import evaluate



PACKAGE_ROOT: Final = Path(__file__).parents[1]
CONFIG_PATH: Final = PACKAGE_ROOT / "config" / "nav2_shadow.yaml"
MISSING_PLANNER_BT_PATH: Final = (
    PACKAGE_ROOT / "behavior_trees" / "navigate_to_pose_shadow_missing_planner.xml"
)


def test_given_shadow_configuration_when_loaded_then_simulated_nav2_uses_shadow_velocity() -> None:
    # Given: Domain 65의 synthetic map·TF fixture만 소비하는 Nav2 설정
    configuration = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))

    # When: planner, controller, BT의 runtime parameter를 읽는다.
    controller = configuration["controller_server"]["ros__parameters"]
    planner = configuration["planner_server"]["ros__parameters"]
    navigator = configuration["bt_navigator"]["ros__parameters"]
    behaviors = configuration["behavior_server"]["ros__parameters"]
    local_costmap = configuration["local_costmap"]["local_costmap"]["ros__parameters"]

    # Then: 모든 Nav2 server는 sim time이고 known planner·controller·shadow BT를 쓴다.
    assert controller["use_sim_time"] is True
    assert planner["use_sim_time"] is True
    assert navigator["use_sim_time"] is True
    assert planner["planner_plugins"] == ["GridBased"]
    assert controller["controller_plugins"] == ["FollowPath"]
    assert isinstance(local_costmap["width"], int)
    assert isinstance(local_costmap["height"], int)
    assert navigator["default_nav_to_pose_bt_xml"].endswith(
        "navigate_to_pose_shadow.xml"
    )
    assert behaviors["behavior_plugins"] == ["spin", "backup", "wait"]


def test_shadow_launch_contains_only_navigation_owners():
    _, nodes, _ = evaluate("go2_nav2", "go2_nav2_shadow.launch.py")
    assert {n["package"] for n in nodes} == {
        "nav2_map_server", "nav2_planner", "nav2_controller", "nav2_bt_navigator",
        "nav2_behaviors", "nav2_lifecycle_manager"}
    for node in nodes:
        if node["package"] in {"nav2_controller", "nav2_behaviors"}:
            remaps = node["action"]._Node__remappings
            from launch.utilities import perform_substitutions
            from launch import LaunchContext
            context = LaunchContext()
            resolved = [(perform_substitutions(context, a), perform_substitutions(context, b))
                        for a, b in remaps]
            assert ("cmd_vel", "/go2_nav2/shadow_cmd_vel") in resolved


def test_behavior_server_is_in_actual_lifecycle_parameters():
    _, nodes, _ = evaluate("go2_nav2", "go2_nav2_shadow.launch.py")
    manager = next(n for n in nodes if n["package"] == "nav2_lifecycle_manager")
    assert "behavior_server" in manager["parameters"][-1]["node_names"]
    assert any(n["executable"] == "behavior_server" for n in nodes)


def test_given_missing_planner_bt_when_parsed_then_it_selects_only_missing_planner() -> None:
    # Given: planner-failure scenario's dedicated BT asset
    root = ElementTree.parse(MISSING_PLANNER_BT_PATH).getroot()

    # When: its compute-path action is located.
    planner = root.find(".//ComputePathToPose")

    # Then: Nav2 receives a machine-consumed MissingPlanner identifier.
    assert planner is not None
    assert planner.attrib["planner_id"] == "MissingPlanner"
