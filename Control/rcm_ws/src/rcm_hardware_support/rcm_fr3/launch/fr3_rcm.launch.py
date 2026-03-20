from launch import LaunchDescription
from launch_param_builder import ParameterBuilder
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
  DeclareLaunchArgument,
  IncludeLaunchDescription,
)

from launch.substitutions import (
  LaunchConfiguration,
  PathJoinSubstitution
)

from launch.conditions import (
  IfCondition,
  UnlessCondition
)

from launch_ros.substitutions import FindPackageShare

from launch.launch_description_sources import PythonLaunchDescriptionSource

def declare_arguments():
  return LaunchDescription(
    [
      DeclareLaunchArgument(
        "servo",
        default_value="false",
        description="Launches MoveIt servo for either Joint Trajectory Controller or CIC, whichever is active first.",
      ),
      DeclareLaunchArgument(
        "launch_moveit",
        default_value="true",
        description="Launch the MoveGroup node",
      ),
      DeclareLaunchArgument(
        "launch_controllers",
        default_value="true",
        description="Launch ros2_control controller manager node, not needed for simulation",
      ),
      DeclareLaunchArgument(
        "launch_rviz",
        default_value="true",
        description="Launch the RViz GUI for MoveIt",
      ),
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
        default_value="true",
        description="Use end-effector poses instead of object poses for input to pick and place",
      ),
      DeclareLaunchArgument(
        "tests_enabled",
        default_value="false",
        description="Run some basic tests on startup",
      ),
      DeclareLaunchArgument(
        "headless",
        default_value='false',
        description='Disable RVIZ GUI, can be launched separately'
      ),
      DeclareLaunchArgument(
        "robot_ip",
        default_value="fake_ip_for_simulation",
        description='Hostname or IP address of the robot. REQUIRED if using real hardware',
      ),
      DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (e.g. Gazebo) time from the /clock topic",
      ),
      DeclareLaunchArgument(
        "initial_joint_controller",
        default_value="fr3_arm_controller",
        choices=[
            "fr3_arm_controller",
            "fr3_arm_compliant_controller",
        ],
        description="Initially loaded robot controller.",
    )
    ]
  )

def generate_launch_description():
  servo = LaunchConfiguration("servo")
  initial_joint_controller = LaunchConfiguration("initial_joint_controller")
  launch_moveit = LaunchConfiguration("launch_moveit")
  launch_controllers = LaunchConfiguration("launch_controllers")
  launch_rviz = LaunchConfiguration("launch_rviz")

  planning_group_name = LaunchConfiguration("planning_group_name")

  end_effector = LaunchConfiguration("end_effector")
  gripper_enabled = LaunchConfiguration("gripper_enabled")
  gripper_plugin = LaunchConfiguration("gripper_plugin")

  pap_enabled = LaunchConfiguration("pap_enabled")
  default_ee_orientation = LaunchConfiguration("default_ee_orientation")
  direct_ee_poses = LaunchConfiguration("direct_ee_poses")

  tests_enabled = LaunchConfiguration("tests_enabled")

  robot_ip = LaunchConfiguration("robot_ip")
  
  use_sim_time = LaunchConfiguration("use_sim_time")

  # Moveit and ros2_control setup
  moveit_setup = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('rcm_fr3_moveit_config'), 'launch', 'moveit.launch.py'])]),
    launch_arguments={
      'robot_ip': robot_ip,
      'use_sim_time': use_sim_time,
      'initial_joint_controller': initial_joint_controller,
      'launch_moveit': launch_moveit,
      'launch_controllers': launch_controllers,
      'launch_rviz': launch_rviz}.items(),
  )

  # RCM MoveIt and End-Effector interfaces
  rcm_main = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('rcm_fr3'), 'launch', 'fr3_rcm_control.launch.py'])]),
    launch_arguments={
      'planning_group_name': planning_group_name,
      'end_effector': end_effector,
      'gripper_enabled': gripper_enabled,
      'pap_enabled': pap_enabled,
      'default_ee_orientation': default_ee_orientation,
      'direct_ee_poses': direct_ee_poses,
      'gripper_plugin': gripper_plugin,
      'tests_enabled': tests_enabled,
      'use_sim_time': use_sim_time}.items()
  )

  # RCM Manager
  rcm_manager = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('rcm_fr3'), 'launch', 'fr3_rcm_manager.launch.py'])]),
  )

  # RCM MoveIt Servo interfaces
  rcm_servo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('rcm_fr3'), 'launch', 'fr3_servo.launch.py'])]),
    launch_arguments={
      'use_sim_time': use_sim_time}.items(),
    condition=IfCondition(servo)
  )

  return LaunchDescription([declare_arguments(), moveit_setup, rcm_main, rcm_manager, rcm_servo])