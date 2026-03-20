# Robot Control

## Prerequisites:
You need to have ROS2 Jazzy, MoveIt2 (we used binary install) and Franka ROS2 (we used v3.2.0)

## How to run the system
### The control side of things:
Run the main control-related functionalities:

**Terminal 1:**

```
cd rcm_ws
source install/setup.bash
ros2 launch rcm_fr3 fr3_rcm.launch.py robot_ip:=<your_robot_ip> initial_joint_controller:=fr3_arm_compliant_controller
```

Start listening and enable voice commands:

**Terminal 2:**

```
cd rcm_ws
source install/setup.bash
ros2 run co_manipulation_pkg voice_command_test_listening
```

**Terminal 3:**

```
cd rcm_ws
source install/setup.bash
ros2 service call /enable_voice_commands std_srvs/srv/SetBool "{data: true}"
```

**Terminal 4:**

```
cd rcm_ws
source install/setup.bash
#If you want to test without a seprarate speech recognition software, you can run for example:
ros2 action send_goal /rcm/voice_command rcm_msgs/action/VoiceCommand '{voice_command: "ALLOW COMPLIANCE XYZ"}'
```
