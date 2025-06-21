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
