"""Domain 65 Nav2 설정·launch·failure BT의 비물리 계약을 검증한다."""

from pathlib import Path
from typing import Final
from xml.etree import ElementTree

import yaml

PACKAGE_ROOT: Final = Path(__file__).parents[1]
CONFIG_PATH: Final = PACKAGE_ROOT / "config" / "nav2_shadow.yaml"
LAUNCH_PATH: Final = PACKAGE_ROOT / "launch" / "go2_nav2_shadow.launch.py"
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


def test_given_shadow_launch_when_inspected_then_only_shadow_cmd_vel_is_exposed() -> None:
    # Given: Domain 65의 loopback-only launch source
    launch_source = LAUNCH_PATH.read_text(encoding="utf-8")

    # When: runtime child와 velocity remapping을 확인한다.
    # Then: Nav2 속도는 isolated topic으로만 가며 control package와 physical topic은 없다.
    assert '"cmd_vel", "/go2_nav2/shadow_cmd_vel"' in launch_source
    assert 'package="nav2_planner"' in launch_source
    assert 'package="nav2_controller"' in launch_source
    assert 'package="nav2_bt_navigator"' in launch_source
    assert 'package="go2_control"' not in launch_source
    assert "/api/sport/request" not in launch_source
    assert "/lowcmd" not in launch_source


def test_given_shadow_launch_when_inspected_then_behavior_server_is_lifecycle_managed() -> None:
    # Given: Domain 65 Nav2 lifecycle composition source
    launch_source = LAUNCH_PATH.read_text(encoding="utf-8")

    # When: runtime package ownership and lifecycle names are inspected.
    # Then: behavior server joins the same bounded Nav2 lifecycle.
    assert 'package="nav2_behaviors"' in launch_source
    assert 'executable="behavior_server"' in launch_source
    assert 'name="behavior_server"' in launch_source
    assert '"behavior_server"' in launch_source


def test_given_missing_planner_bt_when_parsed_then_it_selects_only_missing_planner() -> None:
    # Given: planner-failure scenario's dedicated BT asset
    root = ElementTree.parse(MISSING_PLANNER_BT_PATH).getroot()

    # When: its compute-path action is located.
    planner = root.find(".//ComputePathToPose")

    # Then: Nav2 receives a machine-consumed MissingPlanner identifier.
    assert planner is not None
    assert planner.attrib["planner_id"] == "MissingPlanner"
