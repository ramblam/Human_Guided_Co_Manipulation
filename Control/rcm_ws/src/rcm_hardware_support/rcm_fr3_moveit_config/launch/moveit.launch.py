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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    Shutdown
)
from launch.conditions import UnlessCondition, IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

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

    # Extra launch parameters
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

    launch_moveit_parameter_name = 'launch_moveit'
    launch_controllers_parameter_name = 'launch_controllers'
    launch_rviz_parameter_name = 'launch_rviz'

    launch_moveit = LaunchConfiguration(launch_moveit_parameter_name)
    launch_controllers = LaunchConfiguration(launch_controllers_parameter_name)
    launch_rviz = LaunchConfiguration(launch_rviz_parameter_name)

    # Original launch parameters
    use_sim_time_parameter_name = 'use_sim_time'
    load_gripper_parameter_name = 'load_gripper'
    ee_id_parameter_name = 'ee_id'

    robot_ip_parameter_name = 'robot_ip'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    namespace_parameter_name = 'namespace'

    use_sim_time = LaunchConfiguration(use_sim_time_parameter_name)

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(
        fake_sensor_commands_parameter_name)
    namespace = LaunchConfiguration(namespace_parameter_name)

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

    launch_rviz_arg = DeclareLaunchArgument(
        launch_rviz_parameter_name,
        default_value='true',
        description='Enable RVIZ GUI, can be launched separately'
    )

    launch_moveit_arg = DeclareLaunchArgument(
        launch_moveit_parameter_name,
        default_value='true',
        description='Enable Move Group node, can be launched separately'
    )

    launch_controllers_arg = DeclareLaunchArgument(
        launch_controllers_parameter_name,
        default_value='true',
        description='Disable ros2_control components, can be launched separately'
    )

    # planning_context
    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', 'fr3', 'fr3.urdf.xacro'
    )

    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, ' hand:=', load_gripper,
         ' robot_ip:=', robot_ip, ' ee_id:=', ee_id, ' use_fake_hardware:=', use_fake_hardware,
         ' fake_sensor_commands:=', fake_sensor_commands, ' ros2_control:=true'])

    robot_description = {'robot_description': ParameterValue(
        robot_description_config, value_type=str)}

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

    # Planning Functionality
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugins': ['ompl_interface/OMPLPlanner'],
            'request_adapters': [
                'default_planning_request_adapters/ResolveConstraintFrames',
                'default_planning_request_adapters/ValidateWorkspaceBounds',
                'default_planning_request_adapters/CheckStartStateBounds',
                'default_planning_request_adapters/CheckStartStateCollision',
                                ],
            'response_adapters': [
                'default_planning_response_adapters/AddTimeOptimalParameterization',
                'default_planning_response_adapters/ValidateSolution',
                'default_planning_response_adapters/DisplayMotionPath'
                                  ],
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        'franka_fr3_moveit_config', 'config/ompl_planning.yaml'
    )
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        'rcm_fr3_moveit_config', 'config/fr3_controllers.yaml'
    )
    moveit_controllers = {
        'moveit_switching_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_switching_controller_manager'
                                     '/MoveItSwitchingControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace=namespace,
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_config,
            joint_limits_config,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(launch_moveit)
    )

    # RViz
    rviz_base = os.path.join(get_package_share_directory(
        'franka_fr3_moveit_config'), 'rviz')
    rviz_full_config = os.path.join(rviz_base, 'moveit.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_full_config],
        parameters=[
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_config,
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(launch_rviz)
    )

    # Publish TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='both',
        parameters=[robot_description],
        condition=IfCondition(launch_controllers)
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory('rcm_fr3_controller_config'),
        'config',
        'fr3_rcm_controllers.yaml',
    )
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=namespace,
        parameters=[robot_description, ros2_controllers_path],
        remappings=[('joint_states', 'franka/joint_states')],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
        on_exit=Shutdown(),
        condition=IfCondition(launch_controllers)
    )

    # Load controllers
    """
    load_controllers = []
    for controller in ['fr3_arm_controller', 'joint_state_broadcaster']:
        load_controllers.append(
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'controller_manager', 'spawner', controller,
                    '--controller-manager-timeout', '60',
                    '--controller-manager',
                    PathJoinSubstitution([namespace, 'controller_manager'])
                ],
                output='screen'
            )
        )
    """
    # A more convenient way to load controllers
    controller_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [FindPackageShare('rcm_fr3_controller_config'), 'launch', 'controllers.launch.py'])]),
        launch_arguments={'initial_joint_controller': initial_joint_controller}.items(),
        condition=IfCondition(launch_controllers)
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        parameters=[
            {'source_list': ['franka/joint_states', 'fr3_gripper/joint_states'], 'rate': 30}
        ],
        condition=IfCondition(launch_controllers)
    )

    franka_robot_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=['franka_robot_state_broadcaster'],
        output='screen',
        condition=IfCondition(launch_controllers),
    )

    robot_arg = DeclareLaunchArgument(
        robot_ip_parameter_name,
        description='Hostname or IP address of the robot.')

    namespace_arg = DeclareLaunchArgument(
        namespace_parameter_name,
        default_value='',
        description='Namespace for the robot.'
    )
    use_fake_hardware_arg = DeclareLaunchArgument(
        use_fake_hardware_parameter_name,
        default_value='false',
        description='Use fake hardware')
    fake_sensor_commands_arg = DeclareLaunchArgument(
        fake_sensor_commands_parameter_name,
        default_value='false',
        description="Fake sensor commands. Only valid when '{}' is true".format(
            use_fake_hardware_parameter_name))
    gripper_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
        launch_arguments={'robot_ip': robot_ip,
                          use_fake_hardware_parameter_name: use_fake_hardware,
                          'namespace': namespace}.items(),
                          condition=IfCondition(launch_controllers),
    )
    return LaunchDescription(
        [use_sim_time_arg,
         load_gripper_arg,
         ee_id_arg,
         launch_rviz_arg,
         launch_controllers_arg,
         launch_moveit_arg,
         initial_joint_controller_argument,
         robot_arg,
         namespace_arg,
         use_fake_hardware_arg,
         fake_sensor_commands_arg,
         db_arg,
         rviz_node,
         robot_state_publisher,
         run_move_group_node,
         ros2_control_node,
         joint_state_publisher,
         franka_robot_state_broadcaster,
         gripper_launch_file,
         controller_launch_file
         ]
        #+ load_controllers
    )