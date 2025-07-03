#!/bin/bash
# Simple test script to understand GUI container concepts

echo "=== Simple Gazebo GUI Test ==="
echo ""

# Step 1: Show we're in container
echo "1. Container Check:"
if [ -f /.dockerenv ]; then
    echo "   ✓ Running inside Docker container"
else
    echo "   ✗ Not in container - run this inside devcontainer"
    exit 1
fi

# Step 2: Test X11 forwarding
echo ""
echo "2. X11 Display Test:"
echo "   DISPLAY = $DISPLAY"
if [ -S "/tmp/.X11-unix/X${DISPLAY##*:}" ]; then
    echo "   ✓ X11 socket connected to host"
else
    echo "   ✗ X11 not working"
    exit 1
fi

# Step 3: Test GPU
echo ""
echo "3. GPU Test:"
if command -v nvidia-smi &> /dev/null; then
    echo "   ✓ NVIDIA GPU available"
    nvidia-smi -L | head -1
else
    echo "   ⚠ No GPU (software rendering will be used)"
fi

# Step 4: Test simple GUI
echo ""
echo "4. Simple GUI Test:"
echo "   Opening xclock for 3 seconds..."
xclock &
XCLOCK_PID=$!
sleep 3
kill $XCLOCK_PID 2>/dev/null
echo "   ✓ If you saw a clock window, GUI works!"

# Step 5: ROS2 check
echo ""
echo "5. ROS2 Check:"
source /opt/ros/humble/setup.bash
if command -v ros2 &> /dev/null; then
    echo "   ✓ ROS2 available"
    echo "   TurtleBot3 model: $TURTLEBOT3_MODEL"
else
    echo "   ✗ ROS2 not found"
    exit 1
fi

echo ""
echo "=== All Tests Passed! ==="
echo ""
echo "Ready to launch Gazebo:"
echo "  ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
echo ""
echo "Core concept: Container apps show on host desktop via X11 forwarding"