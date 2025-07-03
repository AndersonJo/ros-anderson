#!/usr/bin/env python3
"""
Launch file to test TurtleBot3 GUI in Docker container
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Set TurtleBot3 model
    os.environ['TURTLEBOT3_MODEL'] = 'burger'
    
    # Gazebo world launch
    gazebo_world_cmd = ExecuteProcess(
        cmd=['ros2', 'launch', 'turtlebot3_gazebo', 'turtlebot3_world.launch.py'],
        output='screen'
    )
    
    # RViz2 launch (delayed to allow Gazebo to start)
    rviz_cmd = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'launch', 'turtlebot3_gazebo', 'turtlebot3_gazebo_rviz.launch.py'],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        gazebo_world_cmd,
        rviz_cmd
    ])