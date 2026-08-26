"""통합 preflight의 필수 topic·node·TF와 timing 후보 기준을 정의한다."""

from pathlib import Path

from bringup.preflight_types import TopicContract


RUNTIME_WRAPPER_PATH = Path(
    "/home/bi-agx1/go2_runtime/go2_agx_ros2_humble_env.sh"
)
STARTUP_GRACE_SECONDS = 15.0
COMMAND_TOPICS = ("/api/sport/request", "/lowcmd")
EXPECTED_NODES = (
    "/base_to_utlidar_lidar_static_tf",
    "/controller_server",
    "/go2_motion_adapter",
    "/go2_obstacle_candidates",
    "/go2_odometry_adapter",
    "/lifecycle_manager_controller",
    "/robot_state_publisher",
)
REQUIRED_TRANSFORMS = (
    ("base", "imu"),
    ("base", "radar"),
    ("base", "front_camera"),
    ("base", "utlidar_lidar"),
    ("odom", "base"),
)
TOPIC_CONTRACTS = (
    TopicContract(
        topic="/lf/lowstate",
        expected_type="unitree_go/msg/LowState",
        expected_frame=None,
        expected_child_frame=None,
        minimum_rate_hz=10.0,
        maximum_gap_seconds=0.5,
    ),
    TopicContract(
        topic="/utlidar/cloud",
        expected_type="sensor_msgs/msg/PointCloud2",
        expected_frame="utlidar_lidar",
        expected_child_frame=None,
        minimum_rate_hz=5.0,
        maximum_gap_seconds=1.0,
    ),
    TopicContract(
        topic="/utlidar/imu",
        expected_type="sensor_msgs/msg/Imu",
        expected_frame="utlidar_imu",
        expected_child_frame=None,
        minimum_rate_hz=100.0,
        maximum_gap_seconds=0.5,
    ),
    TopicContract(
        topic="/utlidar/robot_odom",
        expected_type="nav_msgs/msg/Odometry",
        expected_frame="odom",
        expected_child_frame="base_link",
        minimum_rate_hz=50.0,
        maximum_gap_seconds=0.5,
    ),
    TopicContract(
        topic="/odom",
        expected_type="nav_msgs/msg/Odometry",
        expected_frame="odom",
        expected_child_frame="base",
        minimum_rate_hz=50.0,
        maximum_gap_seconds=0.5,
    ),
    TopicContract(
        topic="/perception/obstacle_candidates",
        expected_type="sensor_msgs/msg/PointCloud2",
        expected_frame="base",
        expected_child_frame=None,
        minimum_rate_hz=5.0,
        maximum_gap_seconds=1.0,
    ),
    TopicContract(
        topic="/local_costmap/costmap",
        expected_type="nav_msgs/msg/OccupancyGrid",
        expected_frame="odom",
        expected_child_frame=None,
        minimum_rate_hz=0.5,
        maximum_gap_seconds=3.0,
    ),
)
