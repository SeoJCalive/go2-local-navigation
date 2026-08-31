"""저장 지도·replay scan·odometry와 단일 AMCL owner를 조합한다.

이 launch는 Domain과 rosbag lifecycle을 소유하지 않는다. Map Server와 AMCL,
기존 read-only mapping scan 및 odometry adapter만 시작하며 planner, controller,
Go2 command node는 시작하지 않는다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Domain 64에서 AMCL이 유일한 map→odom owner가 되도록 구성한다."""
    bringup_share = get_package_share_directory("bringup")
    nav2_share = get_package_share_directory("go2_nav2")
    perception_share = get_package_share_directory("go2_perception")
    scan_launch = os.path.join(
        perception_share,
        "launch",
        "go2_mapping_scan.launch.py",
    )
    odometry_launch = os.path.join(
        bringup_share,
        "launch",
        "go2_odometry_adapter.launch.py",
    )
    parameters = os.path.join(
        nav2_share,
        "config",
        "saved_map_localization.yaml",
    )
    default_map = os.path.join(nav2_share, "maps", "shadow_open.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")
    execution_mode = LaunchConfiguration("execution_mode")
    continuity_profile = LaunchConfiguration("continuity_profile")
    sensor_tf_profile = LaunchConfiguration("sensor_tf_profile")
    scan_projection_profile = LaunchConfiguration("scan_projection_profile")
    map_path = LaunchConfiguration("map")
    sim_time_parameter = {
        "use_sim_time": ParameterValue(use_sim_time, value_type=bool)
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("execution_mode", default_value="onboard"),
            DeclareLaunchArgument(
                "continuity_profile",
                default_value="replay_enforce",
            ),
            DeclareLaunchArgument(
                "sensor_tf_profile",
                default_value="project_default",
            ),
            DeclareLaunchArgument(
                "scan_projection_profile",
                default_value="raw_single",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(scan_launch),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "execution_mode": execution_mode,
                    "sensor_tf_profile": sensor_tf_profile,
                    "scan_projection_profile": scan_projection_profile,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(odometry_launch),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "continuity_profile": continuity_profile,
                }.items(),
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                parameters=[
                    parameters,
                    sim_time_parameter,
                    {"yaml_filename": map_path},
                ],
                output="screen",
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                parameters=[parameters, sim_time_parameter],
                output="screen",
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                parameters=[
                    sim_time_parameter,
                    {
                        "autostart": True,
                        "node_names": ["map_server", "amcl"],
                        "bond_timeout": 4.0,
                    },
                ],
                output="screen",
            ),
        ]
    )
