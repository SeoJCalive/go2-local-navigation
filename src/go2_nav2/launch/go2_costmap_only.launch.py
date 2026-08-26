"""정지 입력 경로와 비동작 Nav2 local costmap owner만 시작한다.

기존 static TF, odometry adapter, obstacle candidate를 재사용한다. Humble의
standalone costmap 대신 clean lifecycle teardown을 제공하는 controller server가
local costmap을 소유하지만, motion adapter와 경로 goal은 시작하지 않는다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Compose read-only project inputs with a standalone local costmap."""
    bringup_share = get_package_share_directory("bringup")
    navigation_share = get_package_share_directory("go2_nav2")
    perception_launch = os.path.join(
        bringup_share,
        "launch",
        "go2_stationary_perception.launch.py",
    )
    odometry_launch = os.path.join(
        bringup_share,
        "launch",
        "go2_odometry_adapter.launch.py",
    )
    params_file = os.path.join(
        navigation_share,
        "config",
        "nav2_non_actuating.yaml",
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(perception_launch)
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(odometry_launch)
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                parameters=[params_file],
                remappings=[
                    ("cmd_vel", "/go2_nav2/costmap_only_cmd_vel_unused"),
                ],
                output="screen",
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_costmap",
                parameters=[
                    {
                        "autostart": True,
                        "node_names": ["controller_server"],
                    }
                ],
                output="screen",
            ),
        ]
    )
