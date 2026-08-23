from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="go2_state_estimation",
                executable="odometry_adapter",
                name="go2_odometry_adapter",
                output="screen",
            ),
        ]
    )
