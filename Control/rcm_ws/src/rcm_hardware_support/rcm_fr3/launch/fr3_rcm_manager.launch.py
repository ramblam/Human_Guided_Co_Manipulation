from launch import LaunchDescription
from launch_ros.actions import Node

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument
)

from launch.substitutions import (
    LaunchConfiguration,
    Command,
    FindExecutable,
)

from launch.conditions import IfCondition

from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

import os
import yaml

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def declare_arguments():
    return LaunchDescription(
        [
          DeclareLaunchArgument(
              "use_sim_time",
              default_value="false",
              description="Use simulation (e.g. Gazebo) time from the /clock topic",
          ),
        ]
    )

def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")

    # RCM Manager node
    rcm_manager = Node(
        package="rcm_manager",
        executable="manager_node",
        output="screen",
        parameters=[
            {
                'manager_plugin': "rcm_manager_cic_plugins/CartesianImpedanceControllerManager",
                'default_controllers': ["fr3_arm_controller"],
                'compliant_controllers': ["fr3_arm_compliant_controller"],
                'stiffness_param_service': "/fr3_arm_compliant_controller/set_parameters_atomically",
                'trans_stiffness_max': 1500.0,
                'trans_stiffness_min': 0.0,
                'rot_stiffness_max': 100.0,
                'rot_stiffness_min': 0.0,
                'use_sim_time': use_sim_time,
            },
        ],
    )

    return LaunchDescription([declare_arguments(), rcm_manager])