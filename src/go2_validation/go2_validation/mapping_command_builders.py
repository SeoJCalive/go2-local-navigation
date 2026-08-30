
"""Domain 63 mapping의 shell 없는 launch·player argv를 조립한다.
이 모듈은 mapping runtime이 소유한 process lifecycle과 분리해, 입력 경로·profile·
재생 속도를 ROS CLI가 소비하는 불변 command tuple로 변환한다. 명령을 실행하지
않으며, 잘못된 재생 속도에는 기존 ``MappingRuntimeError`` reason code를 그대로 쓴다.
"""

from dataclasses import dataclass
from pathlib import Path

from go2_validation.mapping_player_services import MappingRuntimeError


def mapping_bag_play_command(path: Path, playback_rate: float) -> tuple[str, ...]:
    """선택된 raw topic만 single-clock 1.0배속으로 재생하는 argv를 만든다."""
    if not isfinite_positive(playback_rate):
        raise MappingRuntimeError("mapping_playback_rate_invalid", str(playback_rate))
    return (
        "ros2",
        "bag",
        "play",
        str(path),
        "--rate",
        str(playback_rate),
        "--clock",
        "100",
        "--wait-for-all-acked",
        "1000",
        "--delay",
        "1.0",
        "--start-paused",
        "--topics",
        "/utlidar/cloud",
        "/utlidar/robot_odom",
        "--disable-keyboard-controls",
    )


@dataclass(frozen=True, slots=True)
class MappingLaunchConfiguration:
    """Domain 63 mapping launch에 전달하는 불변 profile·SLAM 구성이다."""

    sensor_tf_profile: str
    scan_projection_profile: str = "raw_single"
    execution_mode: str = "onboard"
    continuity_profile: str = "onboard_observe"
    use_response_expansion: bool = True
    do_loop_closing: bool = True
    coarse_search_angle_offset: float = 0.349


def mapping_launch_command(
    configuration: MappingLaunchConfiguration,
) -> tuple[str, ...]:
    """Install-space Todo 12 launch를 sim time으로 시작하는 argv다."""
    return (
        "ros2",
        "launch",
        "go2_nav2",
        "go2_slam_mapping.launch.py",
        "use_sim_time:=true",
        f"execution_mode:={configuration.execution_mode}",
        f"continuity_profile:={configuration.continuity_profile}",
        f"sensor_tf_profile:={configuration.sensor_tf_profile}",
        f"scan_projection_profile:={configuration.scan_projection_profile}",
        f"do_loop_closing:={str(configuration.do_loop_closing).lower()}",
        f"use_response_expansion:={str(configuration.use_response_expansion).lower()}",
        "coarse_search_angle_offset:="
        f"{configuration.coarse_search_angle_offset}",
    )


def isfinite_positive(value: float) -> bool:
    """Player rate가 finite positive인지 표준 float 연산으로 확인한다."""
    return value > 0.0 and value != float("inf")
