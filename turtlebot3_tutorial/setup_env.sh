#!/bin/bash
# Environment setup script
echo "Setting up ROS2 environment..."

if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "✅ ROS2 Humble sourced"
fi

if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "✅ Local workspace sourced"
fi

export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=0

echo "✅ Environment ready!"
echo "🤖 ROS2 version: $(ros2 --version)"
echo "🐢 TurtleBot3 model: $TURTLEBOT3_MODEL"
echo ""
echo "Quick test commands:"
echo "  ros2 topic list"
echo "  ros2 node list" 
echo "  ros2 run simple_test test_node.py"
