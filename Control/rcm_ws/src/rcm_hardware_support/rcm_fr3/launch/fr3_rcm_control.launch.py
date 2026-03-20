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
                "planning_group_name",
                default_value="fr3_arm",
                description="Name of the move group to command (options defined by the robots moveit config)",
            ),
            DeclareLaunchArgument(
                "end_effector",
                default_value="",
                description="Name of the end effector link, attempts to use default if passed an empty string",
            ),
            DeclareLaunchArgument(
                "gripper_enabled",
                default_value="true",
                description="Load the specified gripper plugin",
            ),
            DeclareLaunchArgument(
                "gripper_plugin",
                default_value="rcm_gripper_franka_plugins::FrankaEndEffector",
                description="Which gripper plugin to load, if enabled",
            ),
            DeclareLaunchArgument(
                "pap_enabled",
                default_value="true",
                description="Load the pick and place action server",
            ),
            DeclareLaunchArgument(
                "default_ee_orientation",
                default_value="[1.0, 0.0, 0.0, 0.0]",
                description="Orientation of the end effector when facing down and front towards user [x, y, z, w]",
            ),
            DeclareLaunchArgument(
                "direct_ee_poses",
                default_value="false",
                description="Use end-effector poses instead of object poses for input to pick and place",
            ),
            DeclareLaunchArgument(
                "use_current_orientation",
                default_value="false",
                description="Retrieve the end effector orientation from the starting pose",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (e.g. Gazebo) time from the /clock topic",
            ),
            DeclareLaunchArgument(
                "tests_enabled",
                default_value="false",
                description="Run some basic tests on startup",
            ),
            DeclareLaunchArgument(
                "load_gripper",
                default_value='true',
                description='Whether to load the gripper or not (true or false)'
            ),
            DeclareLaunchArgument(
                "ee_id",
                default_value='franka_hand',
                description='The end-effector id to use. Available options: none, franka_hand, cobot_pump'
            )
        ]
    )

def generate_launch_description():

    planning_group_name = LaunchConfiguration("planning_group_name")
    end_effector = LaunchConfiguration("end_effector")
    gripper_enabled = LaunchConfiguration("gripper_enabled")
    pap_enabled = LaunchConfiguration("pap_enabled")
    default_ee_orientation = LaunchConfiguration("default_ee_orientation")
    direct_ee_poses = LaunchConfiguration("direct_ee_poses")
    use_current_orientation = LaunchConfiguration("use_current_orientation")
    gripper_plugin = LaunchConfiguration("gripper_plugin")
    use_sim_time = LaunchConfiguration("use_sim_time")
    tests_enabled = LaunchConfiguration("tests_enabled")
    load_gripper = LaunchConfiguration("load_gripper")
    ee_id = LaunchConfiguration("ee_id")

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

    joint_limits_yaml = load_yaml(
        'franka_fr3_moveit_config', 'config/fr3_joint_limits.yaml'
    )

    joint_limits_config = {
        'robot_description_planning': joint_limits_yaml
    }

    # RCM MoveIt wrapper node
    rcm_moveit = Node(
        package="rcm_moveit",
        executable="rcm_moveit",
        output="screen",
        parameters=[
            kinematics_config,
            robot_description_semantic,
            joint_limits_config,
            {
            'planning_group_name': planning_group_name,
            'end_effector': end_effector,
            'gripper_enabled': gripper_enabled,
            'gripper_plugin': gripper_plugin,
            'use_sim_time': use_sim_time,
            'tests_enabled': tests_enabled
            },
        ],
    )

    rcm_gripper = Node(
        package="rcm_gripper",
        executable="gripper_controller",
        output="screen",
        parameters=[
            {
            'gripper_plugin': gripper_plugin,
            'use_sim_time': use_sim_time,
            'tests_enabled': tests_enabled
            },    
        ],
        condition=IfCondition(gripper_enabled),
    )

    rcm_pick_and_place = Node(
        package="rcm_pick_and_place",
        executable="pick_and_place_server",
        output="screen",
        parameters=[
            {
            'use_sim_time': use_sim_time,
            'tests_enabled': False,
            'default_ee_orientation': default_ee_orientation,
            'use_current_orientation': use_current_orientation,
            'direct_ee_poses': direct_ee_poses
            },
            
        ],
        condition=IfCondition(pap_enabled),
    )

    return LaunchDescription([declare_arguments(), rcm_moveit, rcm_gripper, rcm_pick_and_place])