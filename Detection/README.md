# Wrist detection:
Vision is used to provide wrist tracking functionality, enabling the robot to track and follow the user’s wrist as they handle the carbon fiber ply together. We used an Intel RealSense D435 camera (but any D400-series camera should work) and Ultralytics YOLO11n-pose model.

## How to run the wrist detection:
The easiest option is to use the provided docker file:
**1. terminal:**
```
xhost +local:docker
# go to the dirctory where the docker files are located:
cd <your_path>/kp_pointer
# (for the first time and when needed:)
sudo docker compose build
sudo docker compose up
```

**2. terminal:**
```
sudo docker exec -it kp_container /bin/bash
# Inside the container run these:
# Source ROS2 Jazzy environment
source /opt/ros/jazzy/setup.bash
# Activate Python virtual environment
source /opt/venv/bin/activate
# Source the workspace (or for the first time build it with colcon build):
source install/setup.bash
# Ensure venv packages are visible to ROS2
export PYTHONPATH=/opt/venv/lib/python3.12/site-packages:$PYTHONPATH	
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=True camera_name:=st_cam
```

**3. terminal:**
```
sudo docker exec -it kp_container /bin/bash
source /opt/ros/jazzy/setup.bash
source /opt/venv/bin/activate
source install/setup.bash
export PYTHONPATH=/opt/venv/lib/python3.12/site-packages:$PYTHONPATH	
ros2 launch pose_keypoint_detector fr3_eye_to_hand_calib.launch.py
```	
**4. terminal:**
```
sudo docker exec -it kp_container /bin/bash
source /opt/ros/jazzy/setup.bash
source /opt/venv/bin/activate
source install/setup.bash
export PYTHONPATH=/opt/venv/lib/python3.12/site-packages:$PYTHONPATH	
ros2 run pose_keypoint_detector yolo_pose_detector
```
