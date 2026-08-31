"""Domain 64 저장 지도 localization runtime 자산 계약을 검증한다."""

from pathlib import Path
from typing import Final

import yaml

PACKAGE_ROOT: Final = Path(__file__).parents[1]
CONFIG_PATH: Final = PACKAGE_ROOT / "config/saved_map_localization.yaml"
LAUNCH_PATH: Final = PACKAGE_ROOT / "launch/go2_saved_map_localization.launch.py"


def test_given_localization_config_when_loaded_then_frames_and_owners_match() -> None:
    # Given: 저장 지도와 project scan·odometry를 소비할 Domain 64 설정
    configuration = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))

    # When: Map Server와 AMCL의 machine-consumed frame 계약을 읽는다.
    map_server = configuration["map_server"]["ros__parameters"]
    amcl = configuration["amcl"]["ros__parameters"]

    # Then: AMCL만 map→odom을 만들고 project canonical base·scan을 사용한다.
    assert map_server["use_sim_time"] is True
    assert amcl["use_sim_time"] is True
    assert amcl["global_frame_id"] == "map"
    assert amcl["odom_frame_id"] == "odom"
    assert amcl["base_frame_id"] == "base"
    assert amcl["scan_topic"] == "/scan"
    assert amcl["tf_broadcast"] is True


def test_given_localization_launch_when_inspected_then_control_path_is_absent() -> None:
    # Given: saved-map localization만 조합해야 하는 runtime launch
    launch_source = LAUNCH_PATH.read_text(encoding="utf-8")

    # When: launch의 package owner와 인자를 조사한다.
    # Then: map/AMCL/input owner는 있고 planner·controller·control owner는 없다.
    assert 'package="nav2_map_server"' in launch_source
    assert 'package="nav2_amcl"' in launch_source
    assert 'name="map_server"' in launch_source
    assert 'name="amcl"' in launch_source
    assert 'DeclareLaunchArgument("map"' in launch_source
    assert 'package="nav2_controller"' not in launch_source
    assert 'package="go2_control"' not in launch_source
    assert "/api/sport/request" not in launch_source
    assert "/lowcmd" not in launch_source
