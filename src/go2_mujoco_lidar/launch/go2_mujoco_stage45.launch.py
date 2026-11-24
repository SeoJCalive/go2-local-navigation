"""공식 Go2 상태를 cloud/scan으로 변환하고 static TF를 한 번 시작한다.

publisher 시작 전 sim domain/loopback을 검사한다. legacy model_path interface는
유지한다. 실제 simulator world 일치 확인과 odom/SLAM 조합은 go2_sim_bringup을 쓴다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from go2_mujoco_lidar.sim_session import simulation_domain


PLANAR_TOPIC = "/simulation/utlidar/cloud_planar_selected"


def _guard(context):
    simulation_domain(
        os.environ,
        LaunchConfiguration("allow_unsafe_simulation").perform(context).lower() == "true",
    )
    return []


def generate_launch_description() -> LaunchDescription:
    perception_share = get_package_share_directory("go2_perception")
    default_model = PathJoinSubstitution(
        [
            EnvironmentVariable("UNITREE_MUJOCO_ROOT"),
            "unitree_robots",
            "go2",
            "scene_terrain.xml",
        ]
    )
    model_path = LaunchConfiguration("model_path")
    horizontal_samples = LaunchConfiguration("horizontal_samples")
    vertical_samples = LaunchConfiguration("vertical_samples")
    vertical_min = LaunchConfiguration("vertical_min")
    vertical_max = LaunchConfiguration("vertical_max")
    environment_only = LaunchConfiguration("environment_only")
    return LaunchDescription(
        [
            DeclareLaunchArgument("model_path", default_value=default_model),
            DeclareLaunchArgument("horizontal_samples", default_value="360"),
            DeclareLaunchArgument("vertical_samples", default_value="8"),
            DeclareLaunchArgument("vertical_min", default_value="-0.2617993877991494"),
            DeclareLaunchArgument("vertical_max", default_value="0.2617993877991494"),
            DeclareLaunchArgument("environment_only", default_value="false"),
            DeclareLaunchArgument("allow_unsafe_simulation", default_value="false",
                                  choices=["true", "false"]),
            DeclareLaunchArgument("raycast_error_limit", default_value="3"),
            OpaqueFunction(function=_guard),
            Node(
                package="go2_mujoco_lidar",
                executable="mujoco_lidar_node",
                parameters=[
                    {
                        "model_path": model_path,
                        "horizontal_samples": horizontal_samples,
                        "vertical_samples": vertical_samples,
                        "vertical_min": vertical_min,
                        "vertical_max": vertical_max,
                        "environment_only": environment_only,
                        "use_sim_time": True,
                        "raycast_error_limit": ParameterValue(
                            LaunchConfiguration("raycast_error_limit"), value_type=int),
                    }
                ],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        perception_share, "launch", "go2_mapping_scan.launch.py"
                    )
                ),
                launch_arguments={
                    "use_sim_time": "true",
                    "execution_mode": "onboard",
                    "sensor_tf_profile": "project_default",
                    "scan_projection_profile": "raw_single",
                    "raw_cloud_topic": PLANAR_TOPIC,
                }.items(),
            ),
        ]
    )
