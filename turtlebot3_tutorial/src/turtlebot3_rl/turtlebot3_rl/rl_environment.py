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
