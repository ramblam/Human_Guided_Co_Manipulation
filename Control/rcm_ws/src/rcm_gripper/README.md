# RCM Gripper Control Packages

## rcm_gripper

Defines the base classes for plugins, and implements some testing. Currently the test function only loads one plugin, to be updated.

```
ros2 run rcm_grippers gripper_test
```

The base class defines the following methods: *open*, *close* and *set_force*. It also defines a simple service interface for ROS integration. An action interface will follow at some point.

## rcm_gripper_dummy_plugins

Implements example plugins for testing. Currently there is a single plugin for `dummy_gripper`:

```
rcm_gripper_dummy_plugins::DummyGripper
```

To see an example of how a plugin can be used, check out the source code for `gripper_test` in `rcm_gripper/src/basic_gripper.cpp`.

## rcm_gripper_franka_plugins

Implements plugins for Franka grippers, primarily Franka Hand. Currently there is a plugin for `franka_hand`:

```
rcm_gripper_franka_plugins::FrankaHand
```


