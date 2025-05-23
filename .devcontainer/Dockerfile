FROM ros:humble

# 기본 패키지 설치
RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    ros-humble-moveit \
    ros-humble-moveit-setup-assistant \
    ros-humble-franka-msgs \
    git wget curl build-essential \
    && rm -rf /var/lib/apt/lists/*

# workspace 설정
RUN mkdir -p /ws/src
WORKDIR /ws
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc