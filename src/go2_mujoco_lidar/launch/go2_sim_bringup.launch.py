"""외부 physics simulator의 실제 world로 local-navigation sim 입력 전체를 시작한다.

simulator_pid는 이미 clean environment에서 실행한 MuJoCo PID다. -s MJCF를
raycast에 그대로 전달하며 model_path는 선택적 기대값이다. simulator/RL은 이 launch의
종료 소유 범위 밖이다. 모든 ROS process는 use_sim_time=true이고 하나라도 종료하면
이 launch가 시작한 나머지를 함께 종료한다. mapping 중 simulator reset 후에는
이 launch 전체를 재시작한다; SLAM map epoch 복구는 지원하지 않는다.

TF tree/owner:
map -> odom: SLAM Toolbox (mapping=true에서만)
odom -> base: go2_odometry_adapter
base -> utlidar_lidar: base_to_utlidar_lidar_static_tf
URDF fixed edges (base -> imu/radar/front_camera 및 다리 고정 링크):
robot_state_publisher. 관절 TF는 joint-state 입력 없이 발행하지 않는다.
/clock: go2_mujoco_lidar만 발행한다. static TF와 scan은 stage45가 한 번 소유한다.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo,
    OpaqueFunction, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from go2_mujoco_lidar.sim_session import simulation_domain, simulator_world


def _start(context):
    unsafe = LaunchConfiguration("allow_unsafe_simulation").perform(context).lower() == "true"
    domain = simulation_domain(os.environ, unsafe)
    world, digest = simulator_world(
        LaunchConfiguration("simulator_pid").perform(context),
        LaunchConfiguration("model_path").perform(context), domain, unsafe,
    )

    def include(package, filename, arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(
                Path(get_package_share_directory(package)) / "launch" / filename)),
            launch_arguments=arguments.items(),
        )

    lidar_arguments = {
        name: LaunchConfiguration(name).perform(context)
        for name in (
            "horizontal_samples", "vertical_samples", "vertical_min", "vertical_max",
            "allow_unsafe_simulation", "raycast_error_limit",
        )
    }
    lidar_arguments["model_path"] = str(world)
    actions = [
        LogInfo(msg=f"simulator/raycast world={world} sha256={digest} domain={domain}"),
        RegisterEventHandler(OnProcessExit(
            on_exit=[EmitEvent(event=Shutdown(reason="sim stack process exited"))])),
        include("go2_mujoco_lidar", "go2_mujoco_stage45.launch.py", lidar_arguments),
        include("go2_inputs", "go2_odometry_adapter.launch.py", {"use_sim_time": "true"}),
    ]
    if LaunchConfiguration("mapping").perform(context).lower() == "true":
        actions.append(include("go2_nav2", "go2_slam_mapping.launch.py", {
            "start_inputs": "false", "use_sim_time": "true",
        }))
    return actions


def generate_launch_description():
    """환경과 world 검증을 끝낸 후에만 ROS node를 생성한다."""
    return LaunchDescription([
        DeclareLaunchArgument("simulator_pid", description="running MuJoCo physics PID"),
        DeclareLaunchArgument("model_path", default_value="", description="optional expected MJCF"),
        DeclareLaunchArgument("mapping", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("allow_unsafe_simulation", default_value="false",
                              choices=["true", "false"]),
        DeclareLaunchArgument("raycast_error_limit", default_value="3"),
        DeclareLaunchArgument("horizontal_samples", default_value="720"),
        DeclareLaunchArgument("vertical_samples", default_value="8"),
        DeclareLaunchArgument("vertical_min", default_value="-0.2617993877991494"),
        DeclareLaunchArgument("vertical_max", default_value="0.2617993877991494"),
        OpaqueFunction(function=_start),
    ])
