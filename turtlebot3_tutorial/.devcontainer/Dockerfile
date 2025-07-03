FROM osrf/ros:humble-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute \
    QT_X11_NO_MITSHM=1 \
    DISPLAY=:0 \
    TZ=Asia/Seoul

ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG USERNAME=user

# Update package lists and install essential packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    # System tools
    bash-completion \
    curl \
    gdb \
    git \
    nano \
    locales \
    openssh-client \
    sudo \
    vim \
    wget \
    tree \
    htop \
    # Build tools - CRITICAL for ROS2 development
    build-essential \
    cmake \
    pkg-config \
    libtool \
    autoconf \
    automake \
    # Python development (fix distutils issue)
    python3-dev \
    python3-pip \
    python3-setuptools \
    python3-distutils \
    python3-venv \
    python3-wheel \
    # ROS2 build tools
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    python3-wstool \
    python3-colcon-argcomplete \
    # GUI support
    libgl1-mesa-glx \
    libglu1-mesa-dev \
    libx11-6 \
    libxext6 \
    libxrender-dev \
    libxrandr2 \
    libxss1 \
    libgconf-2-4 \
    libasound2 \
    libxtst6 \
    libgtk-3-0 \
    libdrm2 \
    libxcomposite1 \
    libxdamage1 \
    libxfixes3 \
    libxcursor1 \
    libxi6 \
    libegl1-mesa \
    libgl1-mesa-dri \
    # X11 tools and utilities
    x11-utils \
    x11-xserver-utils \
    x11-apps \
    xauth \
    mesa-utils \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Setup user configuration
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc

# Setup locale
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Set timezone early to avoid permission issues with devcontainer features
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install ROS2 packages step by step to avoid conflicts
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    # Basic ROS2 packages
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rviz2 \
    ros-humble-rviz-visual-tools \
    ros-humble-tf2-tools \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install TurtleBot3 packages instead of MoveIt2
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3-description \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-navigation2 \
    ros-humble-turtlebot3-cartographer \
    ros-humble-turtlebot3-teleop \
    ros-humble-turtlebot3-example \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install navigation and control packages for RL
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-common \
    ros-humble-nav2-core \
    ros-humble-nav2-msgs \
    ros-humble-nav2-simple-commander \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    ros-humble-tf2-geometry-msgs \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo Classic (avoid Garden conflicts)
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-plugins \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* || echo "Some Gazebo packages may have conflicts, continuing..."

# Install additional Python packages and missing X11 tools
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    python3-distutils-extra \
    python3-catkin-pkg \
    python3-rospkg \
    python3-numpy \
    python3-matplotlib \
    python3-opencv \
    # Additional OpenGL tools (install if available)
    && (apt-get install -y mesa-utils-extra || echo "mesa-utils-extra not available") \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Fix Python distutils for Ubuntu 22.04 with Python 3.10+ and install RL packages
RUN python3 -m pip install --upgrade pip && \
    pip3 --version && \
    # Remove conflicting system packages first
    apt-get update && apt-get remove -y python3-sympy && \
    # Install basic Python packages
    (pip3 install setuptools==58.2.0 --break-system-packages || pip3 install setuptools==58.2.0) && \
    (pip3 install packaging==21.3 --break-system-packages || pip3 install packaging==21.3) && \
    (pip3 install wheel --break-system-packages || pip3 install wheel) && \
    # Install reinforcement learning packages with force reinstall for conflicting packages
    (pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu --break-system-packages --force-reinstall || pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu --force-reinstall) && \
    (pip3 install stable-baselines3[extra] --break-system-packages || pip3 install stable-baselines3[extra]) && \
    (pip3 install gymnasium --break-system-packages || pip3 install gymnasium) && \
    (pip3 install tensorboard --break-system-packages || pip3 install tensorboard) && \
    (pip3 install wandb --break-system-packages || pip3 install wandb) && \
    (pip3 install numpy scipy matplotlib --break-system-packages || pip3 install numpy scipy matplotlib) && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Copy and setup X11 configuration script
COPY setup-x11.sh /usr/local/bin/setup-x11.sh
RUN chmod +x /usr/local/bin/setup-x11.sh

# Create workspace directory
WORKDIR /workspace

# Initialize rosdep (check if already initialized)
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi && rosdep update

# Create runtime directory and fix ownership
RUN mkdir -p /tmp/runtime-user && \
    chmod 755 /tmp/runtime-user && \
    chown -R $USERNAME:$USERNAME /workspace /tmp/runtime-user

USER $USERNAME

# Setup user environment
RUN echo "# ROS2 + TurtleBot3 + RL Development Environment" >> /home/$USERNAME/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "" >> /home/$USERNAME/.bashrc && \
    echo "# TurtleBot3 Model" >> /home/$USERNAME/.bashrc && \
    echo "export TURTLEBOT3_MODEL=burger" >> /home/$USERNAME/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models" >> /home/$USERNAME/.bashrc && \
    echo "" >> /home/$USERNAME/.bashrc && \
    echo "# ROS2 Settings" >> /home/$USERNAME/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> /home/$USERNAME/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> /home/$USERNAME/.bashrc && \
    echo "" >> /home/$USERNAME/.bashrc && \
    echo "# X11 Display settings" >> /home/$USERNAME/.bashrc && \
    echo "export DISPLAY=\${DISPLAY:-:0}" >> /home/$USERNAME/.bashrc && \
    echo "export QT_X11_NO_MITSHM=1" >> /home/$USERNAME/.bashrc && \
    echo "# OpenGL hardware acceleration (fallback to software if needed)" >> /home/$USERNAME/.bashrc && \
    echo "export LIBGL_ALWAYS_INDIRECT=0" >> /home/$USERNAME/.bashrc && \
    echo "export LIBGL_ALWAYS_SOFTWARE=0" >> /home/$USERNAME/.bashrc && \
    echo "export MESA_GL_VERSION_OVERRIDE=3.3" >> /home/$USERNAME/.bashrc && \
    echo "" >> /home/$USERNAME/.bashrc && \
    echo "# Useful aliases" >> /home/$USERNAME/.bashrc && \
    echo "alias cb='colcon build --symlink-install'" >> /home/$USERNAME/.bashrc && \
    echo "alias cbt='colcon test'" >> /home/$USERNAME/.bashrc && \
    echo "alias cbp='colcon build --packages-select'" >> /home/$USERNAME/.bashrc && \
    echo "alias source_ros='source /opt/ros/humble/setup.bash'" >> /home/$USERNAME/.bashrc && \
    echo "alias source_ws='source install/setup.bash'" >> /home/$USERNAME/.bashrc && \
    echo "alias ll='ls -alF'" >> /home/$USERNAME/.bashrc && \
    echo "alias la='ls -A'" >> /home/$USERNAME/.bashrc && \
    echo "alias l='ls -CF'" >> /home/$USERNAME/.bashrc && \
    echo "alias tb3_sim='ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py'" >> /home/$USERNAME/.bashrc && \
    echo "alias tb3_nav='ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True'" >> /home/$USERNAME/.bashrc

# Set the default working directory
WORKDIR /workspace

# Set default environment
ENV SHELL=/bin/bash
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV TURTLEBOT3_MODEL=burger 