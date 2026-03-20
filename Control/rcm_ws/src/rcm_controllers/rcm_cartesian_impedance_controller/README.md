# Cartesian Impedance Controller
[![CI](https://github.com/matthias-mayr/Cartesian-Impedance-Controller/actions/workflows/build_code.yml/badge.svg?branch=master)](https://github.com/matthias-mayr/Cartesian-Impedance-Controller/actions/workflows/build_code.yml)
[![rosdoc](https://github.com/matthias-mayr/Cartesian-Impedance-Controller/actions/workflows/build_docs.yml/badge.svg)](https://matthias-mayr.github.io/Cartesian-Impedance-Controller/)
[![DOI](https://joss.theoj.org/papers/10.21105/joss.05194/status.svg)](https://doi.org/10.21105/joss.05194)
[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Description
This project is an implementation of Cartesian impedance control for robotic manipulators. It is a type of control strategy that sets a dynamic relationship between contact forces and the position of a robot arm, making it suitable for collaborative robots. It is particularily useful when the interesting dimensions in the workspace are in the Cartesian space.

The controller has so far been and tested using the seven degree-of-freedom (DoF) robot arm `Franka Emika Research 3 (FR3)` in simulation.
This controller is used and tested with ROS 2 `humble`.

The implementation consists of a
1. base library that has few dependencies and can e.g. be directly integrated into software such as the DART simulator and a
2. ROS control integration on top of it.

### Short Pitch at ROSCon:
[![IMAGE ALT TEXT](http://img.youtube.com/vi/Q4aPm4O_9fY/0.jpg)](http://www.youtube.com/watch?v=Q4aPm4O_9fY "Cartesian Impedance Controller ROSCon 2022 Lightning Talk")

http://www.youtube.com/watch?v=Q4aPm4O_9fY

## Features

- Configurable stiffness values along all Cartesian dimensions at runtime
- Configurable damping factors along all Cartesian dimensions at runtime
- Change reference pose at runtime
- Apply Cartesian forces and torques at runtime
- Optional filtering of stiffnesses, pose and wrenches for smoother operation
- Handling of joint trajectories with nullspace configurations, e.g. from MoveIt
- Jerk limitation
- Separate base library that can be integrated in non-ROS environments
- Interface to ROS messages and dynamic_reconfigure for easy runtime configuration

![](./res/flowchart.png)

## Torques

The torque signal commanded to the joints of the robot is composed by the superposition of three joint-torque signals:
- The torque calculated for Cartesian impedance control with respect to a Cartesian pose reference in the frame of the EE of the robot (`tau_task`).
- The torque calculated for joint impedance control with respect to a desired configuration and projected in the nullspace of the robot's Jacobian, so it should not affect the Cartesian motion of the robot's end-effector (`tau_ns`).
- The torque necessary to achieve the desired external force command (`cartesian_wrench`), in the frame of the EE of the robot (`tau_ext`).

## Limitations

- Joint friction is not accounted for
- Stiffness and damping values along the Cartesian dimensions are uncoupled
- No built-in gravity compensation for tools or workpieces (can be achieved by commanding a wrench)

## Prerequisites
### Required
- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)

### ROS Controller
We use `RBDyn` to calculate forward kinematics and the Jacobian.

- [ROS](https://www.ros.org/)
- [RBDyn](https://github.com/jrl-umi3218/RBDyn)
- [mc_rbdyn_urdf](https://github.com/jrl-umi3218/mc_rbdyn_urdf)
- [SpaceVecAlg](https://github.com/jrl-umi3218/SpaceVecAlg)

The installation steps for the installation of the non-ROS dependencies are automated in `scripts/install_dependencies.sh`.

## Controller Usage in ROS
Assuming that there is an [initialized colcon workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html) you can clone this repository, install the dependencies and compile the controller.

This allows you to add a controller configuration for the controller type `cartesian_impedance_controller/CartesianImpedanceController` in your `ros_control` configuration.

### Configuration file

**Porting notes**: Not all parameters are handled yet, and explicit Dynamic Reconfigure is no longer needed in ROS2

When using the controller it is a good practice to describe the parameters in a `YAML` file and load it. Usually this is already done by your robot setup - e.g. for [fr3_sim](https://gitlab.tuni.fi/ATME/cognitive-robotics/fr3_sim) that is [here](https://gitlab.tuni.fi/ATME/cognitive-robotics/fr3_sim/-/blob/master/fr3_sim/config/franka_gz_rcm_controllers.yaml?ref_type=heads).

Here is a template of what needs to be in that YAML file that can be adapted:
```YAML
CartesianImpedance_trajectory_controller:
  type: rcm_controllers/CartesianImpedanceController
  joints:                               # Joints to control
    - fr3_joint_1
    - fr3_joint_2
    - fr3_joint_3
    - fr3_joint_4
    - fr3_joint_5
    - fr3_joint_6
    - fr3_joint_7
  end_effector: fr3_hand_tcp            # Link to control arm in
  update_frequency: 1000                # Controller update frequency in Hz
  # Optional parameters - the mentioned values are the defaults
  handle_trajectories: true             # Accept traj., e.g. from MoveIt
  robot_description: /robot_description # In case of a varying name
  wrench_ee_frame: fr3_hand_tcp         # Default frame for wrench commands
  delta_tau_max: 1.0                    # Max. commanded torque diff between steps in Nm
  filtering:                            # Update existing values (0.0 1.0] per s
    nullspace_config: 0.1               # Nullspace configuration filtering
    pose: 0.1                           # Reference pose filtering
    stiffness: 0.1                      # Cartesian and nullspace stiffness
    wrench: 0.1                         # Commanded torque
  verbosity:
    verbose_print: false                # Enables additional prints
```

### Startup

To start up with this controller, eventually the controller spawner needs to load the controller. Typically this is baked into the robot driver. See *fr3_sim* for examples.

### Changing parameters with Dynamic Reconfigure
There is no special Dynamic Reconfigure in ROS2 anymore. Different RQT tools and the command line can be used to change paramters directly, more info coming at some point.

### Changing parameters with ROS messages
In addition to the configuration with `dynamic_reconfigure`, the controller configuration can always be adapted by sending ROS messages. Outside prototyping this is the main way to parameterize it.

The instructions below use `${controller_ns}` for the namespace of your controller. This can for example be `/cartesian_impedance_controller`. If you want to copy the example commands 1:1, you can set an environment variable with `export controller_ns=/<your_namespace>`.

#### End-effector reference pose

**Porting notes:** IMPLEMENTED/UNTESTED, note the ROS2 message format!

New reference poses can be sent to the topic `${controller_ns}/reference_pose`. They are expected to be in the root frame of the robot description which is often `world`. The root frame can be obtained from the parameter server with `rosparam get ${controller_ns}/root_frame`.

To send a new reference pose 0.6m above the root frame pointing into the z-direction of it, execute this:

```bash
ros2 topic pub --once /${controller_ns}/reference_pose geometry_msgs/PoseStamped "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
pose:
  position:
    x: 0.0
    y: 0.0
    z: 0.6
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
```

Most often one wants to have a controlled way to set reference poses. Once can for example use a [trajectory generator for Cartesian trajectories](https://git.cs.lth.se/robotlab/cartesian_trajectory_generator).

#### Cartesian Stiffness

**Porting notes:** IMPLEMENTED, note the ROS2 message format!

In order to set only the Cartesian stiffnesses, once can send a `geometry_msgs/WrenchStamped` to `set_cartesian_stiffness`:

```bash
ros2 topic pub --once /${controller_ns}/set_cartesian_stiffness geometry_msgs/WrenchStamped "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
wrench:
  force:
    x: 300.0
    y: 300.0
    z: 300.0
  torque:
    x: 30.0
    y: 30.0
    z: 30.0"
```

#### Cartesian Damping factors

**Porting notes:** IMPLEMENTED, note the ROS2 message format!

The damping factors can be configured with a `geometry_msgs/WrenchStamped` msg similar to the stiffnesses to the topic `${controller_ns}/set_damping_factors`. Damping factors are in the interval [0.01,2]. These damping factors are additionally applied to the damping rule which is `2*sqrt(stiffness)`.

#### Stiffnesses, damping and nullspace at once
When also setting the nullspace stiffnes, a custom messages of the type `cartesian_impedance_controller/ControllerConfig` is to be sent to `set_config`:

```bash
ros2 topic pub --once /${controller_ns}/set_config cartesian_impedance_controller/ControllerConfig "cartesian_stiffness:
  force: {x: 300.0, y: 300.0, z: 300.0}
  torque: {x: 30.0, y: 30.0, z: 30.0}
cartesian_damping_factors:
  force: {x: 1.0, y: 1.0, z: 1.0}
  torque: {x: 1.0, y: 1.0, z: 1.0}
nullspace_stiffness: 10.0
nullspace_damping_factor: 0.1
q_d_nullspace: [0, 0, 0, 0, 0, 0, 0]"
```

`q_d_nullspace` is the nullspace joint configuration, so the joint configuration that should be achieved if the nullspace allows it. Note that the end-effector pose would deviate if the forward kinematics of this joint configuration do not overlap with it.

#### Cartesian Wrenches

**Porting notes:** IMPLEMENTED/UNTESTED, note the ROS2 message format! 

A Cartesian wrench can be commanded by sending a `geometry_msgs/WrenchStamped` to the topic `${controller_ns}/set_cartesian_wrench`.
Internally the wrenches are applied in the root frame. Therefore wrench messages are transformed into the root frame using `tf`.<br>
**Note:** An empty field `frame_id` is interpreted as end-effector frame since this is the most applicable one when working with a manipulator.<br>
**Note:** The wrenches are transformed into the root frame when they arrive, but not after that. E.g. end-effector wrenches might need to get updated.

```bash
ros2 topic pub --once /${controller_ns}/set_cartesian_wrench geometry_msgs/WrenchStamped "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
wrench:
  force:
    x: 0.0
    y: 0.0
    z: 5.0
  torque:
    x: 0.0
    y: 0.0
    z: 0.0"
```

### Trajectories and MoveIt

**Porting notes:** IMPLEMENTED, see *fr3_sim* for the MoveIt 2 config.

If `handle_trajectories` is not disabled, the controller can also execute trajectories. An action server is spun up at `${controller_ns}/follow_joint_trajectory` and a fire-and-forget topic for the message type `trajectory_msgs/JointTrajectory` is at `${controller_ns}/joint_trajectory`.

In order to use it with `MoveIt` its controller configuration ([example in fr3_sim](https://gitlab.tuni.fi/ATME/cognitive-robotics/fr3_sim/-/blob/master/fr3_moveit_config/config/fr3_rcm_controllers.yaml?ref_type=heads)) needs to look somewhat like this:
```yaml
controller_names:
  - fr3_arm_compliant_controller

fr3_arm_compliant_controller:
  action_ns: follow_joint_trajectory
  type: FollowJointTrajectory
  default: true
  joints:
    - fr3_joint1
    - fr3_joint2
    - fr3_joint3
    - fr3_joint4
    - fr3_joint5
    - fr3_joint6
    - fr3_joint7

```

**Note:** A nullspace stiffness needs to be specified so that the arm also follows the joint configuration and not just the end-effector pose.

## Safety
We have used the controller with Cartesian translational stiffnesses of up to 1000 N/m and experienced it as very stable. It is also stable in singularities.

One additional measure can be to limit the maximum joint torques that can be applied by the robot arm in the URDF. On our KUKA iiwas we limit the maximum torque of each joint to 20 Nm, which allows a human operator to easily interfere at any time just by grabbing the arm and moving it.

When using `iiwa_ros`, these limits can be applied [here](https://github.com/epfl-lasa/iiwa_ros/blob/master/iiwa_description/urdf/iiwa7.xacro#L53-L59). For the Panda they are applied [here](https://github.com/frankaemika/franka_ros/blob/develop/franka_description/robots/panda/joint_limits.yaml#L6). Both arms automatically apply gravity compensation, the limits are only used for the task-level torques on top of that.

## Repository and Contributions
The original ROS1 public code repository is at: https://github.com/matthias-mayr/Cartesian-Impedance-Controller

## Citing this Work
A brief paper about the features and the control theory is accepted at [JOSS](https://joss.theoj.org/papers/10.21105/joss.05194). 
If you are using it or interacting with it, we would appreciate a citation:
```
Mayr et al., (2024). A C++ Implementation of a Cartesian Impedance Controller for Robotic Manipulators. Journal of Open Source Software, 9(93), 5194, https://doi.org/10.21105/joss.05194
```

```bibtex
@article{mayr2024cartesian,
    doi = {10.21105/joss.05194},
    url = {https://doi.org/10.21105/joss.05194},
    year = {2024},
    publisher = {The Open Journal},
    volume = {9},
    number = {93},
    pages = {5194},
    author = {Matthias Mayr and Julian M. Salt-Ducaju},
    title = {A C++ Implementation of a Cartesian Impedance Controller for Robotic Manipulators}, journal = {Journal of Open Source Software}
}
```
### RBDyn Library not found

When starting the controller, this message appears:
```
 [ControllerManager::loadController]: Could not load class 'cartesian_impedance_controller/CartesianImpedanceController': Failed to load library /home/matthias/catkin_ws/devel/lib//libcartesian_impedance_controller_ros.so. Make sure that you are calling the PLUGINLIB_EXPORT_CLASS macro in the library code, and that names are consistent between this macro and your XML. Error string: Could not load library (Pocoexception = libRBDyn.so.1: cannot open shared object file: No such file or directory) /gazebo: [ControllerManager::loadController]: Could not load controller 'CartesianImpedance_trajectory_controller' because controller type 'cartesian_impedance_controller/CartesianImpedanceController' does exist.
 ```

 This happens when a shared library can not be found. They are installed with `scripts/install_dependencies.sh`. The dynamic linker has a cache and we now manually update it by calling `ldconfig` after the installation.

 ### Controller crashes

 Most likely this happens because some parts of the stack like `iiwa_ros` or `RBDyn` were built with `SIMD`/`march=native` being turned on and other parts are not. Everything needs to be built with or without this option in order to have working alignment. This package builds without, because it is otherwise cumbersome for people to ensure that this happens across the whole stack.
