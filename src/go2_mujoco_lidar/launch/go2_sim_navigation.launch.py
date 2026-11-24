"""W03 sensors와 SLAM 또는 저장 지도 AMCL/Nav2를 단일 mode로 실행한다.

simulator/RL/FIFO bridge는 별도 process다. 필수 tf2 prefix, simulator PID, mode,
localization 지도와 pose를 검증한 뒤에만 node를 시작한다. domain lock은 launch
수명 동안 유지한다. mode 전환은 이전 launch의 완전 종료 후 새 launch로 수행한다.
localization 종료는 Nav2 → localization lifecycle을 먼저 FINALIZED로 만든다.
그 뒤 입력에 signal을 보내 /clock 중단 뒤 TF ROS-time timeout이 멈추는 일을 막는다.
"""

import json
import os
from pathlib import Path
import shlex
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.logging import get_logger
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from go2_mujoco_lidar.sim_session import simulation_domain, simulator_world
from go2_nav2.sim_session import initial_pose, map_selection, mode_owners, session_lock, tf2_library

_locks = []


def _shutdown(context, *, managers):
    from go2_nav2.sim_session_guard import shutdown_managed_nodes
    try:
        states = shutdown_managed_nodes(managers)
    except Exception as error:
        get_logger("go2_sim_navigation").error(f"MANAGED_SHUTDOWN_FAILED {error}")
        return []
    return [LogInfo(msg="MANAGED_SHUTDOWN_COMPLETE " + json.dumps(states, sort_keys=True))]


def _start(context):
    def argument(name):
        return LaunchConfiguration(name).perform(context)

    mode = argument("mode")
    owner = mode_owners(mode, argument("map"), argument("initial_pose"))[0]
    domain = simulation_domain(os.environ)
    world, digest = simulator_world(argument("simulator_pid"), "", domain)
    library = tf2_library(argument("tf2_prefix"))
    selected = map_selection(argument("map")) if mode == "localization" else None
    _locks.append(session_lock(domain))
    share = Path(get_package_share_directory("go2_nav2"))
    prefix = [shlex.join([
        sys.executable, "-m", "go2_nav2.tf2_exec", str(library),
    ])]
    parameters = str(share / "config/nav2_mujoco.yaml")

    def include(package, filename, arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(
                Path(get_package_share_directory(package)) / "launch" / filename)),
            launch_arguments=arguments.items())

    actions = [
        LogInfo(msg=json.dumps({
            "mode": mode, "map_odom_owner": owner, "world": str(world),
            "world_sha256": digest, "map": selected, "tf2": str(library),
        }, sort_keys=True)),
        include("go2_mujoco_lidar", "go2_sim_bringup.launch.py", {
            "simulator_pid": argument("simulator_pid"), "mapping": "false",
            **{name: argument(name) for name in (
                "horizontal_samples", "vertical_samples", "vertical_min", "vertical_max")},
        }),
    ]
    guard_parameters = {"mode": mode, "use_sim_time": True}
    if mode == "mapping":
        # This replaces the optional SLAM include in W03 so the tf2 prefix is enforced here.
        actions.append(Node(
            package="slam_toolbox", executable="async_slam_toolbox_node",
            name="slam_toolbox", prefix=prefix, output="screen",
            parameters=[str(share / "config/slam_mapping.yaml"), {"use_sim_time": True}]))
    else:
        for package, executable, name, overrides in [
            ("nav2_map_server", "map_server", "map_server", {"yaml_filename": selected["yaml"]}),
            ("nav2_amcl", "amcl", "amcl", {"set_initial_pose": False, "always_reset_initial_pose": True}),
            ("nav2_planner", "planner_server", "planner_server", {}),
            ("nav2_controller", "controller_server", "controller_server", {}),
            ("nav2_behaviors", "behavior_server", "behavior_server", {}),
            ("nav2_bt_navigator", "bt_navigator", "bt_navigator", {
                "default_nav_to_pose_bt_xml": str(share / "behavior_trees/navigate_to_pose_shadow.xml")}),
        ]:
            actions.append(Node(
                package=package, executable=executable, name=name,
                prefix=prefix, output="screen",
                parameters=[str(share / "config/saved_map_localization.yaml"),
                            parameters, {"use_sim_time": True}, overrides],
                remappings=[("cmd_vel", "/go2_nav2/shadow_cmd_vel")]))
        managers = [
            ("localization", ["map_server", "amcl"], True),
            ("navigation", ["planner_server", "controller_server", "behavior_server", "bt_navigator"], False),
        ]
        for kind, nodes, autostart in managers:
            actions.append(Node(
                package="nav2_lifecycle_manager", executable="lifecycle_manager",
                name=f"lifecycle_manager_{kind}", output="screen",
                parameters=[{"use_sim_time": True, "autostart": autostart, "node_names": nodes}]))
        x, y, yaw = initial_pose(argument("initial_pose"))
        guard_parameters.update({"initial_x": x, "initial_y": y, "initial_yaw": yaw})
    actions.append(Node(
        package="go2_nav2", executable="sim_session_guard",
        output="screen", parameters=[guard_parameters]))
    if mode == "localization":
        # Humble prepends handlers. Register last so this synchronous hook runs
        # before ExecuteProcess forwards Shutdown to any input/clock process.
        actions.append(RegisterEventHandler(OnShutdown(handle_once=True, on_shutdown=[
            OpaqueFunction(function=_shutdown, kwargs={
                "managers": [(kind, names) for kind, names, _ in reversed(managers)]}),
        ])))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("simulator_pid"),
        DeclareLaunchArgument("mode", choices=["mapping", "localization"]),
        DeclareLaunchArgument("map", default_value=""),
        DeclareLaunchArgument("initial_pose", default_value=""),
        DeclareLaunchArgument("tf2_prefix", default_value=os.environ.get("GO2_TF2_PREFIX", "")),
        DeclareLaunchArgument("horizontal_samples", default_value="720"),
        DeclareLaunchArgument("vertical_samples", default_value="8"),
        DeclareLaunchArgument("vertical_min", default_value="-0.2617993877991494"),
        DeclareLaunchArgument("vertical_max", default_value="0.2617993877991494"),
        OpaqueFunction(function=_start),
    ])
