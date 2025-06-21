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
