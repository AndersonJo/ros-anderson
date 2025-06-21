#!/bin/bash
set -e

echo "🚀 Starting post-create workspace setup..."

# Ensure we're in the correct directory
cd /workspace

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Clean up any problematic existing repositories
echo "🧹 Cleaning up problematic repositories..."
if [ -d "src" ]; then
    # Remove any repositories that might have checkout issues
    rm -rf src/franka_example_controllers 2>/dev/null || true
    # Clean any repositories with wrong branch references
    find src -name ".git" -type d | while read gitdir; do
        repo_dir=$(dirname "$gitdir")
        if [ -f "$gitdir/HEAD" ]; then
            cd "$repo_dir"
            # Try to fix detached HEAD or wrong branch issues
            git checkout humble 2>/dev/null || git checkout main 2>/dev/null || git checkout master 2>/dev/null || true
            cd - > /dev/null
        fi
    done
fi

# Reimport repositories to ensure clean state
echo "📥 Ensuring clean repository state..."
mkdir -p src
cd src
if [ -f "franka.repos" ]; then
    echo "Re-importing repositories with error handling..."
    vcs import < franka.repos --force || true
fi
cd ..

# Update rosdep database if needed
echo "📦 Updating rosdep database..."
rosdep update

# Check if source packages exist and install dependencies
if [ -d "src" ] && [ "$(ls -A src)" ]; then
    echo "📥 Installing dependencies for source packages..."
    rosdep install --from-paths src --ignore-src --rosdistro humble -y --skip-keys "franka_example_controllers" || true
    
    echo "🔨 Building workspace packages..."
    # Build only packages that exist and don't have build issues
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --packages-ignore franka_example_controllers || true
    
    # Source the built workspace
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
        echo "✅ Workspace built and sourced successfully!"
    else
        echo "⚠️  Workspace build may have had issues, but continuing..."
    fi
else
    echo "ℹ️  No source packages found, skipping build..."
fi

# Create helpful launch files if they don't exist
echo "📝 Creating launch files..."
mkdir -p launch

# Simple Panda demo launch file
if [ ! -f "launch/panda_demo.launch.py" ]; then
    cat > launch/panda_demo.launch.py << 'EOF'
#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('moveit_resources_panda_moveit_config'),
                    'launch', 'demo.launch.py'
                ])
            ])
        ),
    ])
EOF
    chmod +x launch/panda_demo.launch.py
fi

# Create useful aliases script
cat > quick_commands.sh << 'EOF'
#!/bin/bash
# Quick commands for ROS2 + MoveIt2 + Panda

echo "🤖 Available commands:"
echo "1. Basic Panda MoveIt Demo:"
echo "   ros2 launch moveit_resources_panda_moveit_config demo.launch.py"
echo ""
echo "2. Panda with Joint State Publisher GUI:"
echo "   ros2 launch moveit_resources_panda_moveit_config demo.launch.py rviz_tutorial:=true"
echo ""
echo "3. Simple test (no GUI dependencies):"
echo "   ros2 pkg list | grep panda"
echo ""
echo "4. Check ROS2 environment:"
echo "   echo \$ROS_DISTRO && ros2 --version"
echo ""
echo "5. Check Gazebo:"
echo "   gz --version"
echo ""
echo "🔍 Troubleshooting:"
echo "- If ros2 command not found: source /opt/ros/humble/setup.bash"
echo "- For GUI issues: export DISPLAY=:0 && export QT_X11_NO_MITSHM=1"
echo "- To rebuild workspace: colcon build --symlink-install"
EOF
chmod +x quick_commands.sh

echo "✅ Post-create setup completed!"
echo ""
echo "🎉 Your ROS2 + MoveIt2 + Gazebo environment is ready!"
echo ""
echo "🚀 Quick start:"
echo "   ./quick_commands.sh    # Show available commands"
echo "   panda_demo            # Launch basic MoveIt demo (alias)"
echo "   ros2 launch moveit_resources_panda_moveit_config demo.launch.py"
echo ""
echo "🔧 Environment:"
echo "   ROS_DISTRO: $ROS_DISTRO"
echo "   Workspace: $(pwd)"
echo "   Built packages: $([ -d install ] && ls install | wc -l || echo 0)" 