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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    
    use_sim_time_parameter_name = 'use_sim_time'
    load_gripper_parameter_name = 'load_gripper'
    ee_id_parameter_name = 'ee_id'

    use_sim_time = LaunchConfiguration(use_sim_time_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    ee_id = LaunchConfiguration(ee_id_parameter_name)

    # Command-line arguments

    db_arg = DeclareLaunchArgument(
        'db', default_value='False', description='Database flag'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        use_sim_time_parameter_name,
        default_value='false',
        description='Use sim time, e.g. with Gazebo'
    )

    load_gripper_arg = DeclareLaunchArgument(
        load_gripper_parameter_name,
        default_value='true',
        description='Whether to load the gripper or not (true or false)'
    )
    ee_id_arg = DeclareLaunchArgument(
        ee_id_parameter_name,
        default_value='franka_hand',
        description='The end-effector id to use. Available options: none, franka_hand, cobot_pump'
    )

    # planning_context
    franka_semantic_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', 'fr3', 'fr3.srdf.xacro'
    )

    robot_description_semantic_config = Command(
        [FindExecutable(name='xacro'), ' ',
         franka_semantic_xacro_file, ' hand:=', load_gripper, ' ee_id:=', ee_id]
    )

    robot_description_semantic = {'robot_description_semantic': ParameterValue(
        robot_description_semantic_config, value_type=str)}

    kinematics_yaml = load_yaml(
        'franka_fr3_moveit_config', 'config/kinematics.yaml'
    )

    kinematics_config = {
        'robot_description_kinematics': kinematics_yaml
    }

    # Planning Functionality
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        'franka_fr3_moveit_config', 'config/ompl_planning.yaml'
    )
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)


    # RViz
    rviz_base = os.path.join(get_package_share_directory(
        'franka_fr3_moveit_config'), 'rviz')
    rviz_full_config = os.path.join(rviz_base, 'moveit.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rcm_moveit_rviz2',
        output='log',
        arguments=['-d', rviz_full_config],
        parameters=[
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_config,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription(
        [
            ee_id_arg,
            load_gripper_arg,
            use_sim_time_arg,
            db_arg,
            rviz_node,
        ]
    )
