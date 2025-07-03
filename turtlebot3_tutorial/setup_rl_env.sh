#!/bin/bash
# Setup script for TurtleBot3 + PyTorch + Reinforcement Learning environment

set -e

echo "Setting up TurtleBot3 + PyTorch + RL environment..."

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Install missing packages first
if [ -f "./install-missing-packages.sh" ]; then
    echo "Installing missing packages..."
    ./install-missing-packages.sh
fi

# Setup X11 if in container
if [ -f /usr/local/bin/setup-x11.sh ]; then
    echo "Running X11 setup..."
    /usr/local/bin/setup-x11.sh
fi

# Set TurtleBot3 model
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models

# Build workspace if src directory exists
if [ -d "src" ]; then
    echo "Building ROS2 workspace..."
    colcon build --symlink-install
    
    # Source the workspace
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
        echo "✓ Workspace built and sourced"
    fi
else
    echo "No src directory found, skipping workspace build"
fi

# Test PyTorch installation
echo "Testing PyTorch installation..."
python3 -c "import torch; print(f'PyTorch version: {torch.__version__}'); print(f'CUDA available: {torch.cuda.is_available()}')"

# Test Stable Baselines3
echo "Testing Stable Baselines3..."
python3 -c "import stable_baselines3; print(f'Stable Baselines3 version: {stable_baselines3.__version__}')"

# Test TurtleBot3 packages
echo "Testing TurtleBot3 packages..."
ros2 pkg list | grep turtlebot3 | head -5

echo "Setup complete!"
echo ""
echo "To test GUI:"
echo "1. Run: python3 test_gui.py"
echo "2. Launch TurtleBot3 simulation: ros2 launch test_turtlebot3_gui.launch.py"
echo ""
echo "To start RL training:"
echo "1. Launch simulation: ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
echo "2. Run RL training: python3 src/turtlebot3_rl/scripts/train_rl.py"