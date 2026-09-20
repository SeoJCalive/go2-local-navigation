import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("go2_mujoco_lidar")
    bringup_share = get_package_share_directory("bringup")
    nav2_share = get_package_share_directory("go2_nav2")
    horizontal_samples = LaunchConfiguration("horizontal_samples")
    vertical_samples = LaunchConfiguration("vertical_samples")
    vertical_min = LaunchConfiguration("vertical_min")
    vertical_max = LaunchConfiguration("vertical_max")
    environment_only = LaunchConfiguration("environment_only")
    return LaunchDescription(
        [
            DeclareLaunchArgument("horizontal_samples", default_value="360"),
            DeclareLaunchArgument("vertical_samples", default_value="8"),
            DeclareLaunchArgument("vertical_min", default_value="-0.2617993877991494"),
            DeclareLaunchArgument("vertical_max", default_value="0.2617993877991494"),
            DeclareLaunchArgument("environment_only", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        package_share,
                        "launch",
                        "go2_mujoco_stage62_upright_hold.launch.py",
                    )
                ),
                launch_arguments={
                    "horizontal_samples": horizontal_samples,
                    "vertical_samples": vertical_samples,
                    "vertical_min": vertical_min,
                    "vertical_max": vertical_max,
                    "environment_only": environment_only,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_share, "launch", "go2_odometry_adapter.launch.py")
                ),
                launch_arguments={"use_sim_time": "true"}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_share, "launch", "go2_slam_mapping.launch.py")
                ),
                launch_arguments={
                    "start_inputs": "false",
                    "use_sim_time": "true",
                }.items(),
            ),
        ]
    )
