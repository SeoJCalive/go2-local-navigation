"""진단용 Domain 1 upright hold와 720×8·±15° planar LiDAR 입력을 시작한다.

설치 indoor world를 raycast에 사용한다. physics는 같은 world로 따로 시작해야 하며
controller는 stand pose만 유지한다. RL 주행용 sim 세션과 함께 실행하지 않는다.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("go2_mujoco_lidar"))
    scene_path = LaunchConfiguration("model_path")
    horizontal_samples = LaunchConfiguration("horizontal_samples")
    vertical_samples = LaunchConfiguration("vertical_samples")
    vertical_min = LaunchConfiguration("vertical_min")
    vertical_max = LaunchConfiguration("vertical_max")
    environment_only = LaunchConfiguration("environment_only")
    stage45_launch = package_share / "launch" / "go2_mujoco_stage45.launch.py"
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_path",
                default_value=str(package_share / "scene" / "go2_indoor.xml"),
            ),
            DeclareLaunchArgument("horizontal_samples", default_value="720"),
            DeclareLaunchArgument("vertical_samples", default_value="8"),
            DeclareLaunchArgument("vertical_min", default_value="-0.2617993877991494"),
            DeclareLaunchArgument("vertical_max", default_value="0.2617993877991494"),
            DeclareLaunchArgument("environment_only", default_value="false"),
            SetEnvironmentVariable("ROS_DOMAIN_ID", "1"),
            SetEnvironmentVariable(
                "CYCLONEDDS_URI",
                f"file://{os.path.join(package_share, 'config', 'cyclonedds_loopback.xml')}",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(stage45_launch)),
                launch_arguments={
                    "model_path": scene_path,
                    "horizontal_samples": horizontal_samples,
                    "vertical_samples": vertical_samples,
                    "vertical_min": vertical_min,
                    "vertical_max": vertical_max,
                    "environment_only": environment_only,
                }.items(),
            ),
            Node(
                package="go2_mujoco_lidar",
                executable="go2_mujoco_upright_hold",
                output="screen",
            ),
        ]
    )
