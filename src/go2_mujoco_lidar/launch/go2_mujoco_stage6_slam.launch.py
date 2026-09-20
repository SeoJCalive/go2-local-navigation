"""Compose static MuJoCo Stage 4/5 inputs with one SLAM Toolbox mapping owner."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    """Keep the Stage 4/5 scan and adapter odometry owners outside SLAM."""
    package_share = get_package_share_directory("go2_mujoco_lidar")
    bringup_share = get_package_share_directory("bringup")
    nav2_share = get_package_share_directory("go2_nav2")
    stage45_launch = os.path.join(
        package_share,
        "launch",
        "go2_mujoco_stage45.launch.py",
    )
    odometry_launch = os.path.join(
        bringup_share,
        "launch",
        "go2_odometry_adapter.launch.py",
    )
    slam_mapping_launch = os.path.join(
        nav2_share,
        "launch",
        "go2_slam_mapping.launch.py",
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(stage45_launch)),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(odometry_launch),
                launch_arguments={"use_sim_time": "true"}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_mapping_launch),
                launch_arguments={
                    "start_inputs": "false",
                    "use_sim_time": "true",
                }.items(),
            ),
        ]
    )
