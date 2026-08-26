from pathlib import Path
from typing import Final

import yaml


PACKAGE_ROOT: Final = Path(__file__).parents[1]
CONFIG_PATH: Final = PACKAGE_ROOT / "config" / "nav2_non_actuating.yaml"
COSTMAP_LAUNCH_PATH: Final = PACKAGE_ROOT / "launch" / "go2_costmap_only.launch.py"


def test_given_config_when_loaded_then_frames_and_source_match() -> None:
    with CONFIG_PATH.open(encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)
    local_params = config["local_costmap"]["local_costmap"]["ros__parameters"]
    obstacle = local_params["obstacle_layer"]

    assert local_params["global_frame"] == "odom"
    assert local_params["robot_base_frame"] == "base"
    assert obstacle["obstacle_candidates"]["topic"] == (
        "/perception/obstacle_candidates"
    )
    assert obstacle["obstacle_candidates"]["data_type"] == "PointCloud2"
    assert obstacle["obstacle_candidates"]["marking"] is True
    assert obstacle["obstacle_candidates"]["clearing"] is False
    assert isinstance(local_params["width"], int)
    assert isinstance(local_params["height"], int)


def test_given_controller_when_loaded_then_limits_match_adapter() -> None:
    with CONFIG_PATH.open(encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)
    controller = config["controller_server"]["ros__parameters"]
    follow_path = controller["FollowPath"]

    assert controller["controller_plugins"] == ["FollowPath"]
    assert follow_path["plugin"] == "dwb_core::DWBLocalPlanner"
    assert follow_path["max_vel_x"] <= 0.30
    assert abs(follow_path["min_vel_x"]) <= 0.20
    assert follow_path["max_vel_y"] <= 0.15
    assert abs(follow_path["min_vel_y"]) <= 0.15
    assert follow_path["max_vel_theta"] <= 0.40


def test_given_costmap_only_launch_when_started_then_motion_path_is_absent() -> None:
    launch_source = COSTMAP_LAUNCH_PATH.read_text(encoding="utf-8")

    assert 'package="nav2_controller"' in launch_source
    assert 'executable="controller_server"' in launch_source
    assert "/go2_nav2/costmap_only_cmd_vel_unused" in launch_source
    assert 'package="nav2_costmap_2d"' not in launch_source
    assert 'package="go2_control"' not in launch_source
