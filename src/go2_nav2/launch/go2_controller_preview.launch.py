"""Nav2 controller를 gate가 닫힌 Sport request preview까지 연결한다.

controller의 `cmd_vel`은 프로젝트 내부 candidate topic으로 remap한다. motion
adapter의 두 승인 parameter는 false이며 실제 Sport control publisher를 만들지 않는다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Compose stationary inputs, controller, and non-actuating adapter."""
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
                    ("cmd_vel", "/go2_control/cmd_vel_candidate"),
                ],
                output="screen",
            ),
            Node(
                package="go2_control",
                executable="motion_adapter",
                name="go2_motion_adapter",
                parameters=[
                    {
                        "output_enabled": False,
                        "physical_validation_approved": False,
                    }
                ],
                output="screen",
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_controller",
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
