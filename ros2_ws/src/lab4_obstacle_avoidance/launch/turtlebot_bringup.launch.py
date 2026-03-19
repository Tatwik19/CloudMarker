from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    safety_distance_arg = DeclareLaunchArgument(
        "safety_distance",
        default_value="0.5",
        description="Obstacle distance threshold in meters",
    )

    scan_topic_arg = DeclareLaunchArgument(
        "scan_topic",
        default_value="/robot_09/scan",
        description="LaserScan topic",
    )

    cmd_vel_topic_arg = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="/robot_09/cmd_vel_unstamped",
        description="Velocity command topic (unstamped)",
    )

    cmd_vel_stamped_topic_arg = DeclareLaunchArgument(
        "cmd_vel_stamped_topic",
        default_value="/robot_09/cmd_vel",
        description="Velocity command topic (stamped)",
    )

    forward_speed_arg = DeclareLaunchArgument(
        "forward_speed",
        default_value="0.05",
        description="Forward linear speed in m/s",
    )

    rotation_speed_arg = DeclareLaunchArgument(
        "rotation_speed",
        default_value="0.8",
        description="Angular rotation speed in rad/s",
    )

    debug_logging_arg = DeclareLaunchArgument(
        "debug_logging",
        default_value="True",
        description="Enable periodic debug logs",
    )

    front_window_degrees_arg = DeclareLaunchArgument(
        "front_window_degrees",
        default_value="30.0",
        description="Half-window size for front distance check in degrees",
    )

    front_angle_offset_arg = DeclareLaunchArgument(
        "front_angle_offset_degrees",
        default_value="-30.0",
        description="LiDAR front direction offset in degrees (+CCW, -CW)",
    )

    navigator_node = Node(
        package="lab4_obstacle_avoidance",
        executable="reactive_navigator",
        name="reactive_navigator",
        output="screen",
        parameters=[
            {
                "safety_distance": LaunchConfiguration("safety_distance"),
                "forward_speed": LaunchConfiguration("forward_speed"),
                "rotation_speed": LaunchConfiguration("rotation_speed"),
                "debug_logging": LaunchConfiguration("debug_logging"),
                "front_window_degrees": LaunchConfiguration("front_window_degrees"),
                "front_angle_offset_degrees": LaunchConfiguration("front_angle_offset_degrees"),
            }
        ],
        remappings=[
            ("scan", LaunchConfiguration("scan_topic")),
            ("cmd_vel", LaunchConfiguration("cmd_vel_topic")),
            ("cmd_vel_stamped", LaunchConfiguration("cmd_vel_stamped_topic")),
        ],
    )

    return LaunchDescription(
        [
            safety_distance_arg,
            scan_topic_arg,
            cmd_vel_topic_arg,
            cmd_vel_stamped_topic_arg,
            forward_speed_arg,
            rotation_speed_arg,
            debug_logging_arg,
            front_window_degrees_arg,
            front_angle_offset_arg,
            navigator_node,
        ]
    )