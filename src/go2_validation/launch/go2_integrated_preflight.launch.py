"""
전체 비동작 stack과 시간 제한 preflight observer를 함께 시작한다.

observer가 결과를 저장하고 종료하면 포함된 static TF·odometry·perception·Nav2·
닫힌 motion adapter를 모두 shutdown한다. 실제 command publish는 포함하지 않는다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """기존 controller preview와 observer를 단일 shutdown 경계로 묶는다."""
    navigation_share = get_package_share_directory("go2_nav2")
    controller_launch = os.path.join(
        navigation_share,
        "launch",
        "go2_controller_preview.launch.py",
    )
    duration = LaunchConfiguration("duration_sec")
    run_id = LaunchConfiguration("run_id")
    run_label = LaunchConfiguration("run_label")
    report_path = LaunchConfiguration("report_path")
    observer = Node(
        package="bringup",
        executable="integrated_preflight_observer",
        name="go2_integrated_preflight_observer",
        parameters=[
            {
                "duration_sec": ParameterValue(duration, value_type=int),
                "run_id": run_id,
                "run_label": run_label,
                "report_path": report_path,
            }
        ],
        output="screen",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("duration_sec", default_value="30"),
            DeclareLaunchArgument("run_id", default_value="preflight-unset"),
            DeclareLaunchArgument("run_label", default_value="preflight"),
            DeclareLaunchArgument("report_path", default_value="observer.json"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(controller_launch)
            ),
            observer,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=observer,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason="integrated preflight observer completed"
                            )
                        )
                    ],
                )
            ),
        ]
    )
