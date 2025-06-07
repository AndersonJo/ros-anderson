#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        LogInfo(msg="🚀 Starting ROS2 + MoveIt2 + Gazebo + Franka Test Environment..."),
        
        # Robot state publisher (basic)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': '<robot name="test_robot"><link name="base_link"/></robot>'
            }]
        ),
        
        # Joint state publisher GUI for testing
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/opt/ros/humble/share/rviz_default_plugins/rviz/default.rviz'],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        
        LogInfo(msg="✅ Environment test launched! Check RViz2 and joint_state_publisher_gui"),
    ]) 