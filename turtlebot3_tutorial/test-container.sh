#!/bin/bash
# Test script to run inside the container

echo "Testing container environment..."
echo "=========================================="

# Check if we're in container
if [ -f /.dockerenv ]; then
    echo "✓ Running inside Docker container"
else
    echo "✗ Not running inside container"
    exit 1
fi

# Check ROS2
if [ -f /opt/ros/humble/setup.bash ]; then
    echo "✓ ROS2 Humble found"
    source /opt/ros/humble/setup.bash
    if command -v ros2 &> /dev/null; then
        echo "✓ ros2 command available"
        echo "  ROS_DISTRO: $ROS_DISTRO"
    else
        echo "✗ ros2 command not found"
    fi
else
    echo "✗ ROS2 not found"
fi

# Check DISPLAY
echo "Display: $DISPLAY"
if [ -S "/tmp/.X11-unix/X${DISPLAY##*:}" ]; then
    echo "✓ X11 socket exists"
else
    echo "✗ X11 socket not found"
fi

# Check TurtleBot3 environment
echo "TurtleBot3 model: $TURTLEBOT3_MODEL"

# Check GPU
if command -v nvidia-smi &> /dev/null; then
    echo "✓ nvidia-smi available"
    nvidia-smi -L
else
    echo "⚠ nvidia-smi not available"
fi

# Check basic commands
echo ""
echo "Available commands:"
echo "  gazebo: $(command -v gazebo &> /dev/null && echo 'Available' || echo 'Not available')"
echo "  rviz2: $(command -v rviz2 &> /dev/null && echo 'Available' || echo 'Not available')"
echo "  glxinfo: $(command -v glxinfo &> /dev/null && echo 'Available' || echo 'Not available')"
echo "  xrandr: $(command -v xrandr &> /dev/null && echo 'Available' || echo 'Not available')"

echo ""
echo "Ready to test GUI applications!"