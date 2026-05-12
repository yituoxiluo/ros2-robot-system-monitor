from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    default_config_file = PathJoinSubstitution(
        [FindPackageShare("robot_system_monitor"), "config", "monitor.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config_file,
                description="Path to the monitor parameter file.",
            ),
            Node(
                package="robot_system_monitor",
                executable="device_driver_node",
                name="device_driver_node",
                output="screen",
                parameters=[config_file],
            ),
            Node(
                package="robot_system_monitor",
                executable="communication_manager_node",
                name="communication_manager_node",
                output="screen",
                parameters=[config_file],
            ),
            Node(
                package="robot_system_monitor",
                executable="diagnostic_monitor_node",
                name="diagnostic_monitor_node",
                output="screen",
                parameters=[config_file],
            ),
            Node(
                package="robot_system_monitor",
                executable="logger_node",
                name="logger_node",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
