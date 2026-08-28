"""기존 scan·odometry 입력과 단일 SLAM Toolbox mapping owner를 조합한다."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Domain과 network는 runner가 소유하고 mapping process만 시작한다."""
    bringup_share = get_package_share_directory("bringup")
    nav2_share = get_package_share_directory("go2_nav2")
    perception_share = get_package_share_directory("go2_perception")
    mapping_scan_launch = os.path.join(
        perception_share,
        "launch",
        "go2_mapping_scan.launch.py",
    )
    odometry_launch = os.path.join(
        bringup_share,
        "launch",
        "go2_odometry_adapter.launch.py",
    )
    default_parameters = os.path.join(nav2_share, "config", "slam_mapping.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")
    execution_mode = LaunchConfiguration("execution_mode")
    continuity_profile = LaunchConfiguration("continuity_profile")
    sensor_tf_profile = LaunchConfiguration("sensor_tf_profile")
    scan_projection_profile = LaunchConfiguration("scan_projection_profile")
    slam_parameters = LaunchConfiguration("slam_params_file")
    use_response_expansion = LaunchConfiguration("use_response_expansion")
    do_loop_closing = LaunchConfiguration("do_loop_closing")
    coarse_search_angle_offset = LaunchConfiguration("coarse_search_angle_offset")
    sim_time_parameter = {
        "use_sim_time": ParameterValue(use_sim_time, value_type=bool)
    }
    response_expansion_parameter = {
        "use_response_expansion": ParameterValue(
            use_response_expansion,
            value_type=bool,
        )
    }
    loop_closing_parameter = {
        "do_loop_closing": ParameterValue(
            do_loop_closing,
            value_type=bool,
        )
    }
    coarse_search_angle_offset_parameter = {
        "coarse_search_angle_offset": ParameterValue(
            coarse_search_angle_offset,
            value_type=float,
        )
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("execution_mode", default_value="onboard"),
            DeclareLaunchArgument("continuity_profile", default_value="onboard_observe"),
            DeclareLaunchArgument(
                "sensor_tf_profile",
                default_value="project_default",
            ),
            DeclareLaunchArgument(
                "scan_projection_profile",
                default_value="raw_single",
            ),
            DeclareLaunchArgument("use_response_expansion", default_value="true"),
            DeclareLaunchArgument("do_loop_closing", default_value="true"),
            DeclareLaunchArgument("coarse_search_angle_offset", default_value="0.349"),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=default_parameters,
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mapping_scan_launch),
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
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                parameters=[
                    slam_parameters,
                    sim_time_parameter,
                    response_expansion_parameter,
                    loop_closing_parameter,
                    coarse_search_angle_offset_parameter,
                ],
                output="screen",
            ),
        ]
    )
