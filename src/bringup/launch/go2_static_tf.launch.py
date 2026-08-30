from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from bringup.static_tf_profiles import load_static_tf_profile


def _sensor_tf_node(context: LaunchContext, profile_path: str) -> list[Node]:
    profile_id = LaunchConfiguration("sensor_tf_profile").perform(context)
    execution_mode = LaunchConfiguration("execution_mode").perform(context)
    profile = load_static_tf_profile(Path(profile_path), profile_id, execution_mode)
    translation = tuple(str(value) for value in profile.translation_xyz_m)
    quaternion = tuple(str(value) for value in profile.quaternion_xyzw)
    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_utlidar_lidar_static_tf",
            arguments=[
                "--x",
                translation[0],
                "--y",
                translation[1],
                "--z",
                translation[2],
                "--qx",
                quaternion[0],
                "--qy",
                quaternion[1],
                "--qz",
                quaternion[2],
                "--qw",
                quaternion[3],
                "--frame-id",
                profile.parent_frame,
                "--child-frame-id",
                profile.child_frame,
            ],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    """Canonical URDF와 선택된 sensor TF profile의 단일 owner를 시작한다."""
    description_share = get_package_share_directory("description")
    bringup_share = get_package_share_directory("bringup")
    urdf_path = Path(description_share) / "urdf" / "go2_description.urdf"
    profile_path = Path(bringup_share) / "config" / "static_tf_profiles.yaml"
    robot_description = urdf_path.read_text(encoding="utf-8")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sensor_tf_profile",
                default_value="project_default",
            ),
            DeclareLaunchArgument("execution_mode", default_value="onboard"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
            ),
            OpaqueFunction(
                function=_sensor_tf_node,
                kwargs={"profile_path": str(profile_path)},
            ),
        ]
    )
