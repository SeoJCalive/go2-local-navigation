"""Domain 64 localization process 명령과 격리 경계를 검증한다."""

from pathlib import Path
from typing import Final

import pytest
import rclpy
from go2_validation.localization_runtime_execution import (
    localization_launch_command,
)
from go2_validation.localization_runtime_observer import LocalizationRuntimeObserver
from go2_validation.mapping_command_builders import mapping_bag_play_command

LOOPBACK_CYCLONEDDS_URI: Final = (
    "<CycloneDDS><Domain><General><Interfaces>"
    '<NetworkInterface name="lo" priority="default" multicast="false" />'
    "</Interfaces></General></Domain></CycloneDDS>"
)


def test_given_saved_map_when_launch_command_is_built_then_profiles_are_explicit() -> None:
    # Given: stationary mapping run에서 저장한 지도 fixture
    map_path = Path("data/runs/mapping/project_stationary/artifacts/occupancy.yaml")

    # When: Domain 64 runtime launch argv를 만든다.
    command = localization_launch_command(map_path)

    # Then: AMCL launch와 replay-only 입력 profile이 모두 명시된다.
    assert command[:4] == (
        "ros2",
        "launch",
        "go2_nav2",
        "go2_saved_map_localization.launch.py",
    )
    assert f"map:={map_path}" in command
    assert "use_sim_time:=true" in command
    assert "execution_mode:=onboard" in command
    assert "continuity_profile:=replay_enforce" in command
    assert "sensor_tf_profile:=project_default" in command
    assert "scan_projection_profile:=raw_single" in command


def test_given_stationary_bag_when_player_command_is_reused_then_it_is_paused() -> None:
    # Given: localization 입력과 동일한 stationary raw bag
    bag_path = Path("data/bags/go2_stationary_raw_20260826_1829")

    # When: 검증된 mapping player command를 그대로 조립한다.
    command = mapping_bag_play_command(bag_path, 1.0)

    # Then: 단일 clock과 명시적 Resume 전 pause 경계를 유지한다.
    assert "--clock" in command
    assert "--start-paused" in command
    assert "/utlidar/cloud" in command
    assert "/utlidar/robot_odom" in command


def test_given_localization_observer_when_destroyed_then_registry_remains_mutable(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # Given: Domain 64 observer가 실제 rclpy subscription을 소유한다.
    monkeypatch.setenv("CYCLONEDDS_URI", LOOPBACK_CYCLONEDDS_URI)
    monkeypatch.setenv("ROS_DOMAIN_ID", "64")
    rclpy.init()
    node = LocalizationRuntimeObserver()

    # When: rclpy가 node 내부 entity registry를 정리한다.
    try:
        node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

    # Then: destroy_node가 내부 subscription registry 오류 없이 반환한다.
