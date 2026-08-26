"""
Launch an offline Go2 URDF and static TF visualization in RViz2.

This launch reuses the project static TF launch, publishes synthetic joint states
from the canonical URDF, and opens RViz2 with the checked-in visualization config.
It does not subscribe to live Go2 topics or publish odometry, sensor data, or
motion commands.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Build the offline model, static TF, joint-state, and RViz actions."""
    bringup_share = get_package_share_directory("bringup")
    description_share = get_package_share_directory("description")
    static_tf_launch = os.path.join(bringup_share, "launch", "go2_static_tf.launch.py")
    rviz_config = os.path.join(bringup_share, "rviz", "go2_offline_model.rviz")
    urdf_path = os.path.join(description_share, "urdf", "go2_description.urdf")
    with open(urdf_path, encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    offline_cyclonedds_uri = (
        '<CycloneDDS><Domain><General><Interfaces>'
        '<NetworkInterface name="lo" priority="default" multicast="false" />'
        "</Interfaces></General></Domain></CycloneDDS>"
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                name="CYCLONEDDS_URI",
                value=offline_cyclonedds_uri,
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(static_tf_launch)
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="offline_joint_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
