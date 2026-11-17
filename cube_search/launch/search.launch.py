from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = ParameterValue(
        LaunchConfiguration("use_sim_time"),
        value_type=bool,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use the simulation clock",
        ),

        Node(
            package="cube_search",
            executable="aruco_detector",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "marker_length": 0.07,
                "optical_frame": "camera_rgb_optical_frame",
                "target_frame": "map",
                "show_image": False
            }],
        ),

        Node(
            package="cube_search",
            executable="search_server",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
            }],
        ),
    ])