#!/bin/bash
set -e

cd /ws

# Source ROS environment
source /opt/ros/humble/setup.bash

echo "Starting workspace build process..."

# Update rosdep
echo "Updating rosdep database..."
rosdep update

# Install dependencies for all packages
echo "Installing dependencies..."
rosdep install --from-paths src --ignore-src --rosdistro humble -y || true

# Build dependencies first (geometric_shapes, srdfdom, warehouse_ros)
echo "Building base dependencies..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select \
  geometric_shapes \
  srdfdom \
  warehouse_ros

# Source the workspace
source install/setup.bash

# Build control messages and interfaces
echo "Building control messages..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select \
  control_msgs

# Source again
source install/setup.bash

# Build basic resource packages
echo "Building resource packages..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select \
  moveit_resources_fanuc_description \
  moveit_resources_panda_description \
  moveit_resources_pr2_description \
  moveit_resources_prbt_support \
  moveit_configs_utils

# Source the workspace to make these packages available
source install/setup.bash

# Build moveit_resources configs
echo "Building moveit resource configs..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select \
  moveit_resources_fanuc_moveit_config \
  moveit_resources_panda_moveit_config \
  moveit_resources_prbt_moveit_config \
  moveit_resources_prbt_pg70_support \
  moveit_resources_prbt_ikfast_manipulator_plugin

# Source again
source install/setup.bash

# Build core moveit packages
echo "Building core MoveIt packages..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select \
  moveit_core \
  moveit_ros_planning \
  moveit_ros_move_group \
  moveit_ros_planning_interface \
  moveit_ros_warehouse \
  moveit_kinematics \
  moveit_planners_ompl

# Source again
source install/setup.bash

# Build ros2_control packages
echo "Building ros2_control packages..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select \
  ros2_control \
  ros2_controllers

# Source again
source install/setup.bash

# Build Gazebo control packages
echo "Building Gazebo control packages..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select \
  gazebo_ros2_control

# Source again
source install/setup.bash

# Build ros_gz packages
echo "Building ros_gz packages..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select \
  ros_gz

# Source again
source install/setup.bash

# Build Franka description packages
echo "Building Franka description packages..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select \
  franka_description

# Source again
source install/setup.bash

# Build Franka ROS2 packages
echo "Building Franka ROS2 packages..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select \
  franka_ros2

# Source again
source install/setup.bash

# Build MoveIt Task Constructor packages
echo "Building MoveIt Task Constructor packages..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select \
  moveit_task_constructor_msgs \
  rviz_marker_tools

# Source again
source install/setup.bash

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select \
  moveit_task_constructor_core

# Source again
source install/setup.bash

# Build remaining MTC packages
echo "Building remaining MoveIt Task Constructor packages..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select \
  moveit_task_constructor_capabilities \
  moveit_task_constructor_visualization \
  moveit_task_constructor_demo

# Source again
source install/setup.bash

# Finally build MoveIt2 tutorials
echo "Building MoveIt2 tutorials..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select \
  moveit2_tutorials

# Final source
source install/setup.bash

echo "Build process completed successfully!"

# Setup environment for GUI applications
echo "Setting up environment for GUI applications..."
echo "export DISPLAY=:0" >> ~/.bashrc
echo "export QT_X11_NO_MITSHM=1" >> ~/.bashrc

# Add workspace setup to bashrc for future sessions
echo "source /ws/install/setup.bash" >> ~/.bashrc

echo "Setup completed! You can now use:"
echo "  - ROS2 Humble"
echo "  - Gazebo Garden"
echo "  - MoveIt2"
echo "  - Franka Emika support"
echo "  - All integrated with ros2_control"