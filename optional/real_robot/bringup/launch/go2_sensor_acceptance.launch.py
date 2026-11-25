from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="go2_sensor",
                executable="lidar_acceptance",
                name="lidar_acceptance",
                output="screen",
            ),
            Node(
                package="go2_state_estimation",
                executable="odometry_probe",
                name="go2_odometry_source_probe",
                output="screen",
            ),
        ]
    )
