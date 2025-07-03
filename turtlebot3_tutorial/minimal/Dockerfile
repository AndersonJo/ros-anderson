# Minimal Dockerfile for Gazebo GUI in Container
# Educational version - shows only essential components

FROM osrf/ros:humble-desktop-full

# Install minimal GUI support packages
RUN apt-get update && apt-get install -y \
    x11-apps \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Install TurtleBot3 packages (essential only)
RUN apt-get update && apt-get install -y \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-gazebo \
    && rm -rf /var/lib/apt/lists/*

# Set environment variables for TurtleBot3
ENV TURTLEBOT3_MODEL=burger
ENV ROS_DOMAIN_ID=0

# Create workspace
WORKDIR /workspace

# Setup ROS environment in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "echo 'Simple Gazebo GUI Container Ready!'" >> ~/.bashrc