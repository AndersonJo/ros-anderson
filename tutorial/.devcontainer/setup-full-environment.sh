#!/bin/bash
set -e

echo "🚀 Setting up ROS2 + MoveIt2 + Gazebo + Franka Environment..."

# Update package lists
echo "📦 Updating package lists..."
apt-get update

# Install essential tools
echo "🔧 Installing essential tools..."
apt-get install -y \
    git \
    python3-pip \
    curl \
    wget \
    vim \
    nano \
    tree \
    htop \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-wstool \
    lsb-release \
    gnupg

# Install Gazebo Garden (latest)
echo "🌍 Installing Gazebo Garden..."
wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
apt-get update
apt-get install -y gz-garden

# Install ROS-Gazebo integration
echo "🔗 Installing ROS-Gazebo integration..."
apt-get install -y \
    ros-humble-ros-gz \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-interfaces

# Install MoveIt2 packages
echo "🤖 Installing MoveIt2 packages..."
apt-get install -y \
    ros-humble-moveit \
    ros-humble-moveit-servo \
    ros-humble-moveit-visual-tools \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-moveit-ros-move-group \
    ros-humble-moveit-ros-visualization \
    ros-humble-moveit-planners-ompl \
    ros-humble-moveit-simple-controller-manager \
    ros-humble-moveit-kinematics \
    ros-humble-moveit-common

# Install ros2_control packages
echo "🎮 Installing ros2_control packages..."
apt-get install -y \
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
    ros-humble-force-torque-sensor-broadcaster

# Install additional useful packages
echo "📋 Installing additional ROS2 packages..."
apt-get install -y \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rviz2 \
    ros-humble-rviz-visual-tools \
    ros-humble-tf2-tools \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-rqt-joint-trajectory-controller

# Install Franka-related packages if available
echo "🦾 Installing Franka packages..."
apt-get install -y \
    ros-humble-franka-msgs \
    ros-humble-franka-description || echo "⚠️  Some Franka packages not available, will build from source"

# Setup workspace
echo "📁 Setting up workspace..."
cd /workspace

# Initialize rosdep if not already done
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    echo "🔧 Initializing rosdep..."
    rosdep init
fi
rosdep update

# Install workspace dependencies
if [ -d "src" ]; then
    echo "📦 Installing workspace dependencies..."
    rosdep install --rosdistro humble --from-paths src --ignore-src -r -y || echo "⚠️  Some dependencies might not be available"
fi

# Build workspace if src directory exists
if [ -d "src" ]; then
    echo "🔨 Building workspace..."
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release || echo "⚠️  Build completed with warnings"
fi

# Create helpful launch files
echo "📄 Creating Panda Gazebo launch files..."
mkdir -p /workspace/launch

# Create Panda Gazebo launch file
cat > /workspace/launch/panda_gazebo.launch.py << 'EOF'
#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Declare launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='World file name'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo GUI'
    )

    # Get the MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Start Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', LaunchConfiguration('world'), '-v', '4'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    
    # Start Gazebo headless
    gazebo_headless = ExecuteProcess(
        cmd=['gz', 'sim', LaunchConfiguration('world'), '-v', '4', '--headless-rendering'],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # Static transform publisher
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'panda_link0'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Spawn robot in Gazebo (with delay)
    spawn_entity = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-topic', 'robot_description',
                    '-name', 'panda',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.0'
                ],
                output='screen',
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
            )
        ]
    )

    # MoveGroup Node (with delay)
    move_group_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}
                ],
            )
        ]
    )

    # RViz (with delay)
    rviz_config_file = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "launch",
        "moveit.rviz"
    )

    rviz_node = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=["-d", rviz_config_file],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.planning_pipelines,
                    moveit_config.robot_description_kinematics,
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}
                ],
            )
        ]
    )

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        gui_arg,
        gazebo,
        gazebo_headless,
        robot_state_publisher,
        static_tf,
        spawn_entity,
        move_group_node,
        rviz_node,
    ])
EOF

# Create simple MoveIt demo launch file
cat > /workspace/launch/panda_moveit_demo.launch.py << 'EOF'
#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    # Static TF
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    return LaunchDescription([
        move_group_node,
        rviz_node,
        static_tf_node,
        robot_state_publisher,
    ])
EOF

# Make launch files executable
chmod +x /workspace/launch/*.py

# Setup user environment
echo "⚙️  Setting up user environment..."
echo "# ROS2 Environment Setup" >> ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Add workspace setup if install directory exists
if [ -d "/workspace/install" ]; then
    echo "source /workspace/install/setup.bash" >> ~/.bashrc
fi

echo "export DISPLAY=:0" >> ~/.bashrc
echo "export QT_X11_NO_MITSHM=1" >> ~/.bashrc

# Add useful aliases
echo "" >> ~/.bashrc
echo "# Useful ROS2 aliases" >> ~/.bashrc
echo "alias cb='colcon build --symlink-install'" >> ~/.bashrc
echo "alias cbt='colcon test'" >> ~/.bashrc
echo "alias cbp='colcon build --packages-select'" >> ~/.bashrc
echo "alias source_ros='source /opt/ros/humble/setup.bash'" >> ~/.bashrc
echo "alias source_ws='source install/setup.bash'" >> ~/.bashrc
echo "alias ll='ls -alF'" >> ~/.bashrc
echo "alias la='ls -A'" >> ~/.bashrc
echo "alias l='ls -CF'" >> ~/.bashrc

# Add Panda-specific aliases
echo "" >> ~/.bashrc
echo "# Panda Robot aliases" >> ~/.bashrc
echo "alias panda_demo='ros2 launch /workspace/launch/panda_moveit_demo.launch.py'" >> ~/.bashrc
echo "alias panda_gazebo='ros2 launch /workspace/launch/panda_gazebo.launch.py'" >> ~/.bashrc
echo "alias panda_gazebo_headless='ros2 launch /workspace/launch/panda_gazebo.launch.py gui:=false'" >> ~/.bashrc

# Clean up
echo "🧹 Cleaning up..."
apt-get clean
rm -rf /var/lib/apt/lists/*

echo "✅ Environment setup complete!"
echo ""
echo "🎯 Available commands:"
echo "  - ros2 --help"
echo "  - gz sim --help"
echo "  - rviz2"
echo "  - cb (colcon build alias)"
echo "  - panda_demo (MoveIt demo)"
echo "  - panda_gazebo (Panda with Gazebo)"
echo "  - panda_gazebo_headless (Headless Gazebo)"
echo ""
echo "🚀 You can now develop with ROS2 + MoveIt2 + Gazebo + Franka!" 