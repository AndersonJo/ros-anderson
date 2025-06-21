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
