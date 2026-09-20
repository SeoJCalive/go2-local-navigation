"""Stage 6-1 MuJoCo SLAM source and launch ownership contracts."""

from pathlib import Path
from typing import Final


PACKAGE_ROOT: Final = Path(__file__).resolve().parents[1]
NODE_SOURCE: Final = PACKAGE_ROOT / "src" / "mujoco_lidar_node.cpp"
STAGE6_LAUNCH: Final = PACKAGE_ROOT / "launch" / "go2_mujoco_stage6_slam.launch.py"


def test_given_mujoco_state_when_cloud_is_published_then_matching_source_odometry_is_published() -> None:
    """The simulated cloud and source odometry share one timestamp and contract."""
    source = NODE_SOURCE.read_text(encoding="utf-8")

    assert "#include <nav_msgs/msg/odometry.hpp>" in source
    assert 'kOdomTopic[] = "/utlidar/robot_odom"' in source
    assert 'kOdomFrame[] = "odom"' in source
    assert 'kBaseFrame[] = "base_link"' in source
    assert "make_odometry(stamp, *sport_state_)" in source
    assert "odometry.header.stamp = stamp;" in source
    assert "odometry.header.frame_id = kOdomFrame;" in source
    assert "odometry.child_frame_id = kBaseFrame;" in source
    assert "sport_state.velocity[0]" in source
    assert "sport_state.imu_state.gyroscope[0]" in source


def test_given_stage6_when_started_then_existing_inputs_and_one_slam_owner_are_composed() -> None:
    """Stage 4/5 owns scan, the adapter owns /odom, and SLAM owns map to odom."""
    source = STAGE6_LAUNCH.read_text(encoding="utf-8")

    assert '"go2_mujoco_stage45.launch.py"' in source
    assert '"go2_odometry_adapter.launch.py"' in source
    assert '"go2_slam_mapping.launch.py"' in source
    assert '"start_inputs": "false"' in source
    assert '"use_sim_time": "true"' in source
    assert "go2_mapping_scan.launch.py" not in source
    assert "go2_static_tf.launch.py" not in source
    assert 'package="slam_toolbox"' not in source
    assert "Node(" not in source
