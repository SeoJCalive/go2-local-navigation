from pathlib import Path
from typing import Final


PACKAGE_ROOT: Final = Path(__file__).resolve().parents[1]


def test_body_pose_comes_from_one_sport_state_sample_when_source_is_read() -> None:
    source = (PACKAGE_ROOT / "src" / "mujoco_lidar_node.cpp").read_text(
        encoding="utf-8"
    )

    assert "quaternion[index] = sport_state_->imu_state.quaternion[index];" in source
    assert "sport_state.imu_state.gyroscope[0]" in source
    assert "low_state.imu_state" not in source


def test_simulation_state_timestamp_drives_clock_cloud_and_odometry() -> None:
    source = (PACKAGE_ROOT / "src" / "mujoco_lidar_node.cpp").read_text(
        encoding="utf-8"
    )

    assert "rosgraph_msgs/msg/clock.hpp" in source
    assert "sport_state_->stamp.sec" in source
    assert "clock_publisher_->publish" in source
    assert "state timestamps differ" in source
    assert "const auto stamp = now();" not in source


def test_cloud_publishers_are_reliable_when_source_is_read() -> None:
    source = (PACKAGE_ROOT / "src" / "mujoco_lidar_node.cpp").read_text(
        encoding="utf-8"
    )

    assert "publisher_qos" in source
    assert ".reliable()" in source


def test_launch_keeps_onboard_mode_and_uses_simulation_height_floor() -> None:
    source = (PACKAGE_ROOT / "launch" / "go2_mujoco_stage45.launch.py").read_text(
        encoding="utf-8"
    )

    assert '"execution_mode": "onboard"' in source
    assert '"converter_min_height": "-0.10"' in source
