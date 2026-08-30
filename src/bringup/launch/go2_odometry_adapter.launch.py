from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    continuity_profile = LaunchConfiguration("continuity_profile")
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "continuity_profile",
                default_value="onboard_observe",
            ),
            Node(
                package="go2_state_estimation",
                executable="odometry_adapter",
                name="go2_odometry_adapter",
                parameters=[
                    {
                        "continuity_profile": ParameterValue(
                            continuity_profile,
                            value_type=str,
                        ),
                        "use_sim_time": ParameterValue(
                            use_sim_time,
                            value_type=bool,
                        )
                    }
                ],
                output="screen",
            ),
        ]
    )
