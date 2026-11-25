"""Domain 65의 합성 map·TF·odom만 소비하는 비물리 Nav2 stack을 조합한다.

controller 출력은 inert `/go2_nav2/shadow_cmd_vel`로만 내보내며 AMCL, SLAM,
Go2 control과 Unitree command component는 시작하지 않는다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Map Server와 Nav2 global·local runtime을 단일 lifecycle로 구성한다."""
    nav2_share = get_package_share_directory("go2_nav2")
    parameters = os.path.join(nav2_share, "config", "nav2_shadow.yaml")
    default_map = os.path.join(nav2_share, "maps", "shadow_open.yaml")
    behavior_tree = os.path.join(
        nav2_share,
        "behavior_trees",
        "navigate_to_pose_shadow.xml",
    )
    map_path = LaunchConfiguration("map")
    sim_time = {"use_sim_time": True}
    return LaunchDescription(
        [
            DeclareLaunchArgument("map", default_value=default_map),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                parameters=[parameters, sim_time, {"yaml_filename": map_path}],
                output="screen",
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                parameters=[parameters, sim_time],
                output="screen",
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                parameters=[parameters, sim_time],
                remappings=[("cmd_vel", "/go2_nav2/shadow_cmd_vel")],
                output="screen",
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                parameters=[
                    parameters,
                    sim_time,
                    {"default_nav_to_pose_bt_xml": behavior_tree},
                ],
                output="screen",
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                parameters=[parameters, sim_time],
                remappings=[("cmd_vel", "/go2_nav2/shadow_cmd_vel")],
                output="screen",
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                parameters=[
                    sim_time,
                    {
                        "autostart": True,
                        "node_names": [
                            "map_server",
                            "planner_server",
                            "controller_server",
                            "behavior_server",
                            "bt_navigator",
                        ],
                    },
                ],
                output="screen",
            ),
        ]
    )
