FROM osrf/ros:humble-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute \
    QT_X11_NO_MITSHM=1 \
    DISPLAY=:0

ARG USER_UID=1001
ARG USER_GID=1001
ARG USERNAME=user

# Install essential packages and GUI support
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        bash-completion \
        curl \
        gdb \
        git \
        nano \
        locales \
        openssh-client \
        python3-colcon-argcomplete \
        python3-rosdep \
        python3-colcon-common-extensions \
        sudo \
        vim \
        wget \
        x11-apps \
        mesa-utils \
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
        libxrandr2 \
        libegl1-mesa \
        libgl1-mesa-dri \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Setup user configuration
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc

# 로케일 설정
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Install Gazebo Garden
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# ROS dependency 설치 (Gazebo + MoveIt2 + ros2_control + Franka 지원)
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        # Gazebo 및 시뮬레이션 관련
        gz-garden \
        ros-humble-ros-gz \
        ros-humble-ros-gz-sim \
        ros-humble-ros-gz-bridge \
        ros-humble-ros-gz-interfaces \
        # MoveIt2 관련 패키지
        ros-humble-moveit \
        ros-humble-moveit-servo \
        ros-humble-moveit-visual-tools \
        ros-humble-moveit-ros-planning-interface \
        ros-humble-moveit-ros-move-group \
        ros-humble-moveit-ros-visualization \
        ros-humble-moveit-planners-ompl \
        ros-humble-moveit-simple-controller-manager \
        ros-humble-moveit-kinematics \
        ros-humble-moveit-common \
        # ros2_control 관련
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-controller-interface \
        ros-humble-controller-manager \
        ros-humble-control-msgs \
        ros-humble-hardware-interface \
        ros-humble-joint-state-broadcaster \
        ros-humble-joint-trajectory-controller \
        ros-humble-position-controllers \
        ros-humble-effort-controllers \
        ros-humble-velocity-controllers \
        ros-humble-gripper-controllers \
        ros-humble-force-torque-sensor-broadcaster \
        # 기타 필수 패키지
        ros-humble-xacro \
        ros-humble-robot-state-publisher \
        ros-humble-joint-state-publisher \
        ros-humble-joint-state-publisher-gui \
        ros-humble-rviz2 \
        ros-humble-rviz-visual-tools \
        ros-humble-tf2-tools \
        ros-humble-rqt \
        ros-humble-rqt-common-plugins \
        ros-humble-rqt-joint-trajectory-controller \
        # 개발 도구
        python3-coverage \
        python3-rospkg \
        python3-pip \
        python3-setuptools \
        python3-wheel \
        # 빌드 도구
        build-essential \
        cmake \
        libeigen3-dev \
        libboost-all-dev \
        libassimp-dev \
        libconsole-bridge-dev \
        libfcl-dev \
        libompl-dev \
        liburdfdom-dev \
        liburdfdom-headers-dev \
        liboctomap-dev \
        # GUI 및 시각화 라이브러리
        freeglut3-dev \
        libqt5opengl5-dev \
        qtbase5-dev \
        qtdeclarative5-dev \
        libqt5svg5-dev \
        # 추가 유용한 도구들
        python3-vcstool \
        python3-wstool \
        tree \
        htop \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

# Tutorial Repos
COPY .devcontainer/tutorial.repos ./src/tutorial.repos
RUN vcs import src < ./src/tutorial.repos

# Mark Git directories as safe to avoid ownership warnings
RUN git config --global --add safe.directory '*'

# Copy and setup script
COPY .devcontainer/setup.sh ./setup.sh
RUN chmod +x ./setup.sh

# Install dependencies and build packages
RUN rosdep update && \
    rosdep install --rosdistro humble --from-paths src --ignore-src -r -y && \
    source /opt/ros/humble/setup.bash && \
    ./setup.sh

# Fix ownership of workspace
RUN chown -R $USERNAME:$USERNAME /ws

USER $USERNAME

# Add workspace setup to user's bashrc
RUN echo "source /ws/install/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "export DISPLAY=:0" >> /home/$USERNAME/.bashrc \
    && echo "export QT_X11_NO_MITSHM=1" >> /home/$USERNAME/.bashrc

# Set default working directory
WORKDIR /ws 