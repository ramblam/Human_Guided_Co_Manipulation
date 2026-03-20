from launch import LaunchDescription
from launch_param_builder import ParameterBuilder
from launch_ros.actions import Node

from launch.actions import (
  DeclareLaunchArgument
)

from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration
)


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

    # This sets the update rate and planning group name for the acceleration limiting filter of MoveIt Servo
    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name = {"planning_group_name": "fr3_arm"}
    
    # Get parameters for the Servo node
    servo_params = {
        "moveit_servo": ParameterBuilder("rcm_servo")
        .yaml("config/fr3_gazebo_config.yaml")
        .to_dict()
    }

    servo_compliant_params = {
        "moveit_servo": ParameterBuilder("rcm_servo")
        .yaml("config/fr3_gazebo_compliant_config.yaml")
        .to_dict()
    }

    franka_semantic_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots',
        'fr3',
        'fr3.srdf.xacro'
    )

    robot_description_semantic_config = Command(
        [FindExecutable(name='xacro'), ' ',
         franka_semantic_xacro_file, ' hand:=true']
    )

    robot_description_semantic = {'robot_description_semantic': ParameterValue(
        robot_description_semantic_config, value_type=str)}

    kinematics_yaml = load_yaml(
        'franka_fr3_moveit_config', 'config/kinematics.yaml'
    )

    # MoveIt Servo node
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="rcm_servo_node",
        parameters=[
            servo_params,
            acceleration_filter_update_period,
            planning_group_name,
            robot_description_semantic,
            kinematics_yaml,
            {'use_sim_time': use_sim_time}
        ],
        output="screen",
    )

    # MoveIt Servo node
    servo_compliant_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="rcm_servo_node_compliant",
        parameters=[
            servo_compliant_params,
            acceleration_filter_update_period,
            planning_group_name,
            robot_description_semantic,
            kinematics_yaml,
            {'use_sim_time': use_sim_time}
        ],
        output="screen",
    )

    return LaunchDescription([declare_arguments(), servo_node, servo_compliant_node])
