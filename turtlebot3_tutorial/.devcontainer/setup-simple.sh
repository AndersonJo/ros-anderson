#!/bin/bash

echo "🚀 Simple DevContainer Setup for ROS2 Development"

# Source ROS2 environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "✅ ROS2 Humble environment sourced"
else
    echo "⚠️ ROS2 Humble not found, but continuing..."
fi

# Set TurtleBot3 model
export TURTLEBOT3_MODEL=burger
echo "✅ TurtleBot3 model set to: $TURTLEBOT3_MODEL"

# Create basic workspace structure
echo "📁 Creating basic workspace structure..."
mkdir -p src build install log
echo "✅ Workspace directories created"

# Update rosdep if available
if command -v rosdep &> /dev/null; then
    echo "🔧 Updating rosdep..."
    rosdep update
    echo "✅ rosdep updated"
else
    echo "⚠️ rosdep not available, skipping..."
fi

# Create simple test package structure
echo "🛠️ Creating test package structure..."
mkdir -p src/simple_test/{scripts,launch,config}

# Create a simple Python test script
cat > src/simple_test/scripts/test_node.py << 'EOF'
#!/usr/bin/env python3
"""
Simple test node for ROS2 environment verification
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.publisher_ = self.create_publisher(String, 'test_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0
        self.get_logger().info('Test node started!')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS2! Count: {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

chmod +x src/simple_test/scripts/test_node.py

# Create simple launch file
cat > src/simple_test/launch/test.launch.py << 'EOF'
#!/usr/bin/env python3
"""
Simple test launch file
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simple_test',
            executable='test_node.py',
            name='test_node',
            output='screen'
        )
    ])
EOF

# Create environment setup script
cat > setup_env.sh << 'EOF'
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
EOF

chmod +x setup_env.sh

# Test basic functionality
echo "🧪 Testing basic functionality..."

# Test Python environment
python3 -c "
import sys
print(f'✅ Python {sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}')

try:
    import numpy as np
    print(f'✅ NumPy {np.__version__}')
except ImportError:
    print('⚠️ NumPy not available')

try:
    import rclpy
    print('✅ rclpy available')
except ImportError:
    print('⚠️ rclpy not available')
"

# Try to build a minimal workspace if colcon is available
if command -v colcon &> /dev/null; then
    echo "🏗️ Testing colcon build..."
    # Only build if we have proper setup
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        colcon build --symlink-install --continue-on-error || echo "⚠️ Build had issues, but continuing..."
    else
        echo "⚠️ Skipping build - ROS2 not properly sourced"
    fi
else
    echo "⚠️ colcon not available"
fi

echo ""
echo "🎉 Simple Setup Complete!"
echo "===========================================" 
echo "📋 Next Steps:"
echo "1. Source environment: source setup_env.sh"
echo "2. Test ROS2: ros2 topic list"
echo "3. Build workspace: colcon build"
echo "4. Test packages: ros2 run simple_test test_node.py"
echo ""
echo "🛠️ For TurtleBot3 development:"
echo "1. Install TurtleBot3 packages (if not already installed)"
echo "2. Test simulation: ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
echo "3. Use teleop: ros2 run turtlebot3_teleop teleop_keyboard"
echo ""
echo "Ready for development! 🚀" 