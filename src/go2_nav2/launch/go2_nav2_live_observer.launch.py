"""실제 Go2 입력·AMCL·Nav2를 물리 출력 없이 함께 기동한다.

저장 지도 localization launch가 scan, odometry, Map Server와 AMCL을 소유한다.
Nav2 velocity는 inert topic으로만 remap하며 goal·Go2 control node는 시작하지 않는다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Domain 0 real-time localization과 no-goal Nav2 lifecycle을 조합한다."""
    nav2_share = get_package_share_directory("go2_nav2")
    localization_launch = os.path.join(
        nav2_share,
        "launch",
        "go2_saved_map_localization.launch.py",
    )
    parameters = os.path.join(nav2_share, "config", "nav2_shadow.yaml")
    behavior_tree = os.path.join(
        nav2_share,
        "behavior_trees",
        "navigate_to_pose_shadow.xml",
    )
    default_map = os.path.join(nav2_share, "maps", "shadow_open.yaml")
    map_path = LaunchConfiguration("map")
    wall_time = {"use_sim_time": False}
    return LaunchDescription(
        [
            DeclareLaunchArgument("map", default_value=default_map),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(localization_launch),
                launch_arguments={
                    "map": map_path,
                    "use_sim_time": "false",
                    "execution_mode": "onboard",
                    "continuity_profile": "onboard_observe",
                    "sensor_tf_profile": "project_default",
                    "scan_projection_profile": "raw_single",
                }.items(),
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                parameters=[parameters, wall_time],
                output="screen",
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                parameters=[parameters, wall_time],
                remappings=[("cmd_vel", "/go2_nav2/shadow_cmd_vel")],
                output="screen",
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                parameters=[
                    parameters,
                    wall_time,
                    {"default_nav_to_pose_bt_xml": behavior_tree},
                ],
                output="screen",
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                parameters=[parameters, wall_time],
                remappings=[("cmd_vel", "/go2_nav2/shadow_cmd_vel")],
                output="screen",
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                parameters=[
                    wall_time,
                    {
                        "autostart": True,
                        "node_names": [
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
