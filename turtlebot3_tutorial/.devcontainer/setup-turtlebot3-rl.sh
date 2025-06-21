#!/bin/bash

echo "🚀 Setting up TurtleBot3 + Reinforcement Learning Environment"

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Set TurtleBot3 model
export TURTLEBOT3_MODEL=burger

# Check if we're in a workspace
if [ ! -f "src" ]; then
    echo "📁 Creating workspace structure..."
    mkdir -p src
fi

cd /workspace

echo "📦 Importing TurtleBot3 repositories (optional, as we already have apt packages)..."
if [ -f ".devcontainer/turtlebot3_rl.repos" ]; then
    # Only import if we want latest development versions
    # vcs import src < .devcontainer/turtlebot3_rl.repos || echo "⚠️ Some repositories may already exist"
    echo "✅ Repository file found (using apt packages for stability)"
else
    echo "⚠️ No .repos file found, using apt packages only"
fi

echo "🔧 Installing additional dependencies..."
sudo apt update
rosdep update
# rosdep install --from-paths src --ignore-src -r -y || echo "⚠️ Some dependencies may not be available"

echo "🛠️ Creating simple RL environment package structure..."
mkdir -p src/turtlebot3_rl/{launch,config,scripts,src}

# Create a simple RL environment package
cat > src/turtlebot3_rl/package.xml << 'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypelocation="http://www.w3.org/2001/XMLSchema-instance"?>
<package format="3">
  <name>turtlebot3_rl</name>
  <version>0.0.1</version>
  <description>TurtleBot3 Reinforcement Learning Environment</description>
  <maintainer email="user@example.com">user</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>turtlebot3_gazebo</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF

# Create setup.py
cat > src/turtlebot3_rl/setup.py << 'EOF'
from setuptools import setup

package_name = 'turtlebot3_rl'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/turtlebot3_rl_env.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='TurtleBot3 Reinforcement Learning Environment',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rl_environment = turtlebot3_rl.rl_environment:main',
            'simple_navigator = turtlebot3_rl.simple_navigator:main',
        ],
    },
)
EOF

# Create resource directory
mkdir -p src/turtlebot3_rl/resource
touch src/turtlebot3_rl/resource/turtlebot3_rl

# Create Python package directory
mkdir -p src/turtlebot3_rl/turtlebot3_rl
touch src/turtlebot3_rl/turtlebot3_rl/__init__.py

# Create the launch file that's referenced in setup.py
cat > src/turtlebot3_rl/launch/turtlebot3_rl_env.launch.py << 'EOF'
#!/usr/bin/env python3
"""
TurtleBot3 RL Environment Launch File
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get TurtleBot3 packages
    turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    
    # Launch TurtleBot3 in empty world for RL training
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            turtlebot3_gazebo, '/launch/empty_world.launch.py'
        ])
    )
    
    # Spawn TurtleBot3
    spawn_turtlebot3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            turtlebot3_gazebo, '/launch/spawn_turtlebot3.launch.py'
        ])
    )

    return LaunchDescription([
        gazebo_world,
        spawn_turtlebot3,
    ])
EOF

# Create a simple RL environment node
cat > src/turtlebot3_rl/turtlebot3_rl/rl_environment.py << 'EOF'
#!/usr/bin/env python3
"""
Simple TurtleBot3 RL Environment Node
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np

class TurtleBot3RLEnvironment(Node):
    def __init__(self):
        super().__init__('turtlebot3_rl_environment')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Environment state
        self.laser_data = None
        self.position = None
        
        self.get_logger().info('TurtleBot3 RL Environment initialized!')
    
    def scan_callback(self, msg):
        """Process laser scan data"""
        self.laser_data = np.array(msg.ranges)
        
    def odom_callback(self, msg):
        """Process odometry data"""
        self.position = msg.pose.pose.position
    
    def get_state(self):
        """Get current environment state"""
        if self.laser_data is not None:
            # Simple state: front, left, right distances
            front = np.min(self.laser_data[0:30])
            left = np.min(self.laser_data[60:120])
            right = np.min(self.laser_data[240:300])
            return [front, left, right]
        return [0.0, 0.0, 0.0]
    
    def take_action(self, action):
        """Execute action"""
        twist = Twist()
        
        if action == 0:  # Forward
            twist.linear.x = 0.2
        elif action == 1:  # Turn left
            twist.angular.z = 0.5
        elif action == 2:  # Turn right
            twist.angular.z = -0.5
        else:  # Stop
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    env = TurtleBot3RLEnvironment()
    
    try:
        rclpy.spin(env)
    except KeyboardInterrupt:
        pass
    
    env.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

# Create a simple navigator node
cat > src/turtlebot3_rl/turtlebot3_rl/simple_navigator.py << 'EOF'
#!/usr/bin/env python3
"""
Simple TurtleBot3 Navigator using basic obstacle avoidance
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.laser_data = None
        self.get_logger().info('Simple Navigator started!')
    
    def scan_callback(self, msg):
        """Process laser scan data"""
        self.laser_data = np.array(msg.ranges)
    
    def control_loop(self):
        """Simple obstacle avoidance control"""
        if self.laser_data is None:
            return
        
        twist = Twist()
        
        # Get distances
        front_distance = np.min(self.laser_data[0:30])
        left_distance = np.min(self.laser_data[60:120])
        right_distance = np.min(self.laser_data[240:300])
        
        obstacle_threshold = 0.5
        
        if front_distance > obstacle_threshold:
            # Move forward
            twist.linear.x = 0.2
        else:
            # Turn away from obstacles
            if left_distance > right_distance:
                twist.angular.z = 0.5  # Turn left
            else:
                twist.angular.z = -0.5  # Turn right
        
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    navigator = SimpleNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

echo "🏗️ Building workspace..."
colcon build --symlink-install --continue-on-error

echo "🔗 Sourcing workspace..."
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "✅ Workspace sourced successfully"
else
    echo "⚠️ Build may have failed, but continuing..."
fi

echo "🧪 Creating test launch file..."
cat > test_turtlebot3_simple.launch.py << 'EOF'
#!/usr/bin/env python3
"""
Simple TurtleBot3 Gazebo launch for RL testing
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # TurtleBot3 model
    turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    
    # Launch TurtleBot3 in empty world
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            turtlebot3_gazebo, '/launch/empty_world.launch.py'
        ])
    )
    
    # Spawn TurtleBot3
    spawn_turtlebot3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            turtlebot3_gazebo, '/launch/spawn_turtlebot3.launch.py'
        ])
    )

    return LaunchDescription([
        gazebo_world,
        spawn_turtlebot3,
    ])
EOF

echo ""
echo "🎉 TurtleBot3 + RL Environment Setup Complete!"
echo ""
echo "🚀 Quick Start Commands:"
echo "  source install/setup.bash"
echo "  export TURTLEBOT3_MODEL=burger"
echo ""
echo "📋 Test Commands:"
echo "  # Launch TurtleBot3 in Gazebo"
echo "  ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
echo ""
echo "  # Or use simple test launch"
echo "  ros2 launch test_turtlebot3_simple.launch.py"
echo ""
echo "  # Teleop control"
echo "  ros2 run turtlebot3_teleop teleop_keyboard"
echo ""
echo "  # View topics"
echo "  ros2 topic list"
echo ""
echo "📝 Next Steps:"
echo "  1. Test TurtleBot3 simulation"
echo "  2. Create RL training environment"
echo "  3. Implement simple navigation task"
echo ""
echo "📁 RL Package created at: src/turtlebot3_rl/"
echo "Ready for reinforcement learning development! 🤖🧠" 