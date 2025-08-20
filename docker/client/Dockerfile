# ------------------------------------------------------------
# Dockerfile generated based on ros2-oranor repository
# ------------------------------------------------------------

# ROS 2 Jazzy on Ubuntu Noble
FROM ubuntu:noble-20241015

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Install base dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    tmux \
    gnupg2 \
    lsb-release \
    iproute2 \
    iputils-ping \    
    net-tools \      
    traceroute \
    dnsutils \
    nano \
    software-properties-common \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS 2 repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# Install ROS 2 packages 
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-opencv \
    python3-vcstool \
    ros-${ROS_DISTRO}-turtlesim \
    ros-${ROS_DISTRO}-rmw-zenoh-cpp \
    ros-${ROS_DISTRO}-vision-opencv \
    ros-${ROS_DISTRO}-demo-nodes-cpp && \
    rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Create and setup workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws/src/

# Setup environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc

COPY ./custom_action_interface ./custom_action_interfaces/
COPY /custom_action_client ./custom_action_client