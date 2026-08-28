"""Compose the domain61 fault fixture with the non-actuating mapping inputs."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


LOOPBACK_CYCLONEDDS_URI = (
    '<CycloneDDS><Domain><General><Interfaces>'
    '<NetworkInterface name="lo" priority="default" multicast="false" />'
    "</Interfaces></General></Domain></CycloneDDS>"
)


def generate_launch_description() -> LaunchDescription:
    """Launch only synthetic inputs and non-actuating derived-output owners."""
    bringup_share = get_package_share_directory("bringup")
    perception_share = get_package_share_directory("go2_perception")
    mapping_launch = os.path.join(perception_share, "launch", "go2_mapping_scan.launch.py")
    odometry_launch = os.path.join(bringup_share, "launch", "go2_odometry_adapter.launch.py")
    fixture = Node(
        package="go2_validation",
        executable="fault_fixture",
        name="go2_fault_fixture",
        parameters=[
            {
                "scenario_id": LaunchConfiguration("scenario_id"),
                "fault_kind": LaunchConfiguration("fault_kind"),
                "reason_code": LaunchConfiguration("reason_code"),
                "recovery_deadline_ns": ParameterValue(LaunchConfiguration("recovery_deadline_ns"), value_type=int),
                "restart_attempt": ParameterValue(LaunchConfiguration("restart_attempt"), value_type=bool),
            }
        ],
        output="screen",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    execution_mode = LaunchConfiguration("execution_mode")
    continuity_profile = LaunchConfiguration("continuity_profile")
    return LaunchDescription(
        [
            SetEnvironmentVariable(name="CYCLONEDDS_URI", value=LOOPBACK_CYCLONEDDS_URI),
            DeclareLaunchArgument("scenario_id", default_value="unset"),
            DeclareLaunchArgument("fault_kind", default_value="empty_cloud"),
            DeclareLaunchArgument("reason_code", default_value="EMPTY_CLOUD"),
            DeclareLaunchArgument("recovery_deadline_ns", default_value="1000000000"),
            DeclareLaunchArgument("restart_attempt", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("execution_mode", default_value="onboard"),
            DeclareLaunchArgument(
                "continuity_profile",
                default_value="replay_enforce",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mapping_launch),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "execution_mode": execution_mode,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(odometry_launch),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "continuity_profile": continuity_profile,
                }.items(),
            ),
            Node(package="go2_perception", executable="obstacle_candidates", name="go2_obstacle_candidates", parameters=[{"use_sim_time": True}], output="screen"),
            fixture,
        ]
    )
