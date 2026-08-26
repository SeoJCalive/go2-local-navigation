"""Start existing static TF and the read-only stationary perception node only."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Compose the project static TF owner with obstacle candidate publication."""
    bringup_share = get_package_share_directory("bringup")
    static_tf_launch = os.path.join(
        bringup_share,
        "launch",
        "go2_static_tf.launch.py",
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(static_tf_launch)),
            Node(
                package="go2_perception",
                executable="obstacle_candidates",
                name="go2_obstacle_candidates",
                output="screen",
            ),
        ]
    )
