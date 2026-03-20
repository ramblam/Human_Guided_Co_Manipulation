#!/bin/bash
set -e

# Source ROS2 Jazzy environment
source /opt/ros/jazzy/setup.bash

# Activate Python virtual environment
source /opt/venv/bin/activate

# Source your ROS2 workspace
if [ -f /up/ros2env/install/setup.bash ]; then
    source /up/ros2env/install/setup.bash
fi

# Ensure venv packages are visible to ROS2
export PYTHONPATH=/opt/venv/lib/python3.12/site-packages:$PYTHONPATH

# Drop into an interactive shell
exec "$@"

