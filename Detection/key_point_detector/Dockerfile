# ROS2 Jazzy on Ubuntu 24.04
FROM ros:jazzy

# Environment variables
ENV YOLO_CONFIG_DIR="/tmp"

# Install system dependencies
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
    python3-pip python3-venv python3-dev \
    ros-${ROS_DISTRO}-desktop \
    ros-dev-tools \
    python3-rosdep \
    ros-${ROS_DISTRO}-librealsense2* \
    ros-${ROS_DISTRO}-realsense2-* \
    ros-${ROS_DISTRO}-rqt* \
    ros-${ROS_DISTRO}-tf2 \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-geometry-msgs && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Create workspace
RUN mkdir -p /up/ros2env/src
COPY requirements.txt /up/ros2env
WORKDIR /up/ros2env

# Create a virtual environment
RUN python3 -m venv /opt/venv

# Upgrade pip etc. inside venv and install Python deps
RUN /opt/venv/bin/pip install --upgrade pip setuptools wheel packaging && \
    /opt/venv/bin/pip install --no-cache-dir -r requirements.txt && \
    rm requirements.txt

# ROS tools that still need Python packages
RUN /opt/venv/bin/pip install empy catkin_pkg lark pyyaml
COPY entrypoint.sh /up/ros2env/entrypoint.sh
RUN chmod +x /up/ros2env/entrypoint.sh

# Default shell
CMD ["bash"]