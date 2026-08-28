"""
검증된 mapping cloud를 공식 2D scan converter에 연결한다.

이 launch는 map frame이나 control interface를 소유하지 않는다. 선택된 static TF
profile로 validated cloud를 message timestamp의 project ``base`` frame으로 변환한다.
"""

import os
from pathlib import Path
from typing import Final

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml

from go2_perception.mapping_scan_profiles import (
    JsonValue,
    MappingScanProfile,
    load_mapping_scan_profile,
)


RAW_INPUT_REMAP: Final = ("cloud_in", "/go2_mapping/cloud_validated")


def _converter_parameters(
    scan_parameters: dict[str, JsonValue],
    profile: MappingScanProfile,
) -> dict[str, JsonValue]:
    converter_parameters = scan_parameters.copy()
    if profile.converter_min_height is not None:
        converter_parameters["min_height"] = profile.converter_min_height
    if profile.converter_queue_size is not None:
        converter_parameters["queue_size"] = profile.converter_queue_size
    return converter_parameters


def _projection_nodes(
    context: LaunchContext,
    config_path: str,
    scan_parameters: dict[str, JsonValue],
) -> list[Node]:
    profile_id = LaunchConfiguration("scan_projection_profile").perform(context)
    execution_mode = LaunchConfiguration("execution_mode").perform(context)
    profile = load_mapping_scan_profile(Path(config_path), profile_id, execution_mode)
    converter_parameters = _converter_parameters(scan_parameters, profile)
    use_sim_time = LaunchConfiguration("use_sim_time")
    sim_time_parameter = {
        "use_sim_time": ParameterValue(use_sim_time, value_type=bool)
    }
    nodes: list[Node] = []
    if profile.accumulator_enabled:
        nodes.append(
            Node(
                package="go2_perception",
                executable="mapping_cloud_accumulator",
                name="go2_mapping_cloud_accumulator",
                parameters=[
                    {
                        "frame_limit": profile.frame_limit,
                        "emit_every": profile.emit_every,
                        "input_qos_depth": profile.input_qos_depth,
                        "retry_queue_capacity": profile.retry_queue_capacity,
                        "target_frame": profile.accumulator_target_frame,
                        "output_topic": profile.accumulator_output_topic,
                    },
                    sim_time_parameter,
                ],
                output="screen",
            )
        )
    input_remap = (
        RAW_INPUT_REMAP
        if not profile.accumulator_enabled
        else ("cloud_in", profile.converter_input_topic)
    )
    nodes.append(
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan",
            parameters=[converter_parameters, sim_time_parameter],
            remappings=[input_remap, ("scan", "/scan")],
            output="screen",
        )
    )
    return nodes


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("bringup")
    perception_share = get_package_share_directory("go2_perception")
    static_tf_launch = os.path.join(
        bringup_share,
        "launch",
        "go2_static_tf.launch.py",
    )
    scan_config = os.path.join(perception_share, "config", "mapping_scan.yaml")
    with open(scan_config, encoding="utf-8") as config_file:
        scan_document = yaml.safe_load(config_file)
    scan_parameters = scan_document["mapping_scan"]["pointcloud_to_laserscan"][
        "ros__parameters"
    ]
    use_sim_time = LaunchConfiguration("use_sim_time")
    sensor_tf_profile = LaunchConfiguration("sensor_tf_profile")
    execution_mode = LaunchConfiguration("execution_mode")
    sim_time_parameter = {
        "use_sim_time": ParameterValue(use_sim_time, value_type=bool)
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("execution_mode", default_value="onboard"),
            DeclareLaunchArgument(
                "sensor_tf_profile",
                default_value="project_default",
            ),
            DeclareLaunchArgument(
                "scan_projection_profile",
                default_value="raw_single",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(static_tf_launch),
                launch_arguments={
                    "sensor_tf_profile": sensor_tf_profile,
                    "execution_mode": execution_mode,
                }.items(),
            ),
            Node(
                package="go2_perception",
                executable="mapping_cloud_gate",
                name="go2_mapping_cloud_gate",
                parameters=[sim_time_parameter],
                output="screen",
            ),
            OpaqueFunction(
                function=_projection_nodes,
                kwargs={
                    "config_path": scan_config,
                    "scan_parameters": scan_parameters,
                },
            ),
        ]
    )
