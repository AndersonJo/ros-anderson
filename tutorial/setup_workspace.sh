#!/bin/bash
set -e

echo "🏗️  Setting up Franka + MoveIt2 workspace..."

# Create src directory if it doesn't exist
mkdir -p src

# Download Franka packages
echo "📥 Downloading Franka packages..."
if [ -f ".devcontainer/franka.repos" ]; then
    vcs import src < .devcontainer/franka.repos
else
    echo "⚠️  franka.repos not found, skipping Franka package download"
fi

# Download existing tutorial packages if tutorial.repos exists
if [ -f ".devcontainer/tutorial.repos" ]; then
    echo "📥 Downloading tutorial packages..."
    vcs import src < .devcontainer/tutorial.repos
fi

# Install dependencies
echo "📦 Installing workspace dependencies..."
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --rosdistro humble --from-paths src --ignore-src -r -y

# Build workspace
echo "🔨 Building workspace..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "✅ Workspace setup complete!"
echo ""
echo "📋 Next steps:"
echo "1. source install/setup.bash"
echo "2. ros2 launch panda_moveit_config demo.launch.py (if available)"
echo "3. rviz2 (to visualize)"
echo "4. gz sim (to start Gazebo)" 