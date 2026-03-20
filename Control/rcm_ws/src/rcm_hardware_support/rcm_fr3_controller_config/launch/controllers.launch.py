#  Copyright (c) 2024 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

# This file is an adapted version of
# https://github.com/ros-planning/moveit_resources/blob/ca3f7930c630581b5504f3b22c40b4f82ee6369d/panda_moveit_config/launch/demo.launch.py

import os
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    OpaqueFunction,
    RegisterEventHandler,
    DeclareLaunchArgument,
    ExecuteProcess,
    Shutdown
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    FindExecutable,
)
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

from launch.conditions import (
    UnlessCondition,
    IfCondition
)

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None
    

def controller_setup(context, initial_joint_controller):

    # Spawn controllers
    def controller_spawner(controllers, active=True):
        controller_spawner_timeout="60"
        inactive_flags = ["--inactive"] if not active else []
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                controller_spawner_timeout,
            ]
            + inactive_flags
            + controllers,
        )

    controllers_active = [
        "joint_state_broadcaster"
    ]
    controllers_inactive = [
        "fr3_arm_controller",
        "fr3_arm_compliant_controller",
    ]

    controllers_active.append(initial_joint_controller.perform(context))
    controllers_inactive.remove(initial_joint_controller.perform(context))


    controller_spawners = [
        controller_spawner(controllers_active),
        controller_spawner(controllers_inactive, active=False),
    ]

    return controller_spawners


def generate_launch_description():

    initial_joint_controller_name = 'initial_joint_controller'

    initial_joint_controller = LaunchConfiguration(initial_joint_controller_name)

    initial_joint_controller_argument = DeclareLaunchArgument(
        initial_joint_controller_name,
        default_value="fr3_arm_controller",
        choices=[
            "fr3_arm_controller",
            "fr3_arm_compliant_controller",
        ],
        description="Initially loaded robot controller.",
    )

    controller_spawners = OpaqueFunction(
        function=controller_setup,
        args=[initial_joint_controller])


    return LaunchDescription([   
        initial_joint_controller_argument,
        controller_spawners,
    ])
