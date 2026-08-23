import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    description_share = get_package_share_directory("description")
    urdf_path = os.path.join(description_share, "urdf", "go2_description.urdf")
    with open(urdf_path, encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_utlidar_lidar_static_tf",
                arguments=[
                    "--x",
                    "0.28945",
                    "--y",
                    "0.0",
                    "--z",
                    "-0.046825",
                    "--roll",
                    "0.0",
                    "--pitch",
                    "2.8782",
                    "--yaw",
                    "0.0",
                    "--frame-id",
                    "base",
                    "--child-frame-id",
                    "utlidar_lidar",
                ],
            ),
        ]
    )
