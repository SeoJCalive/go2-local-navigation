"""저장 지도 localization runtime 자산 계약을 검증한다."""

from pathlib import Path
from typing import Final

import yaml
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from launch_test_support import evaluate



PACKAGE_ROOT: Final = Path(__file__).parents[1]
CONFIG_PATH: Final = PACKAGE_ROOT / "config/saved_map_localization.yaml"


def test_given_localization_config_when_loaded_then_frames_and_owners_match() -> None:
    # Given: 저장 지도와 project scan·odometry를 소비할 설정
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


def test_localization_launch_has_only_input_and_localization_owners():
    _, nodes, includes = evaluate("go2_nav2", "go2_saved_map_localization.launch.py")
    assert {n["package"] for n in nodes} == {
        "nav2_map_server", "nav2_amcl", "nav2_lifecycle_manager"}
    assert len(includes) == 2


def test_external_localization_disables_both_inputs():
    _, nodes, includes = evaluate("go2_nav2", "go2_saved_map_localization.launch.py",
                                  {"start_inputs": "false"})
    assert not includes
    assert {n["executable"] for n in nodes} == {"map_server", "amcl", "lifecycle_manager"}
