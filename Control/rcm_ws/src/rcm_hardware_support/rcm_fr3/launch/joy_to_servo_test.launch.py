import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument
)
from launch.substitutions import (
    LaunchConfiguration
)

def declare_arguments():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "force_mode",
                default_value="false",
                description="Use wrench commands instead of twist (cic only)",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (e.g. Gazebo) time from the /clock topic",
            ),
        ]
    )

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    force_mode = LaunchConfiguration("force_mode")

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}]
    )

    joy_to_servo_node = Node(
        package="rcm_servo",
        executable="servo_controller_test",
        name="joy_to_servo_node",
        output="screen",
        parameters=[{'use_sim_time': use_sim_time, 'force_mode': force_mode}]
    )

    return LaunchDescription(
        [
            declare_arguments(),
            joy_node,
            joy_to_servo_node,
        ]
    )