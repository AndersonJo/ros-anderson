#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='false',  # Set to false by default for headless operation
        description='Start Gazebo GUI'
    )

    # Get the MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Start Gazebo (headless mode by default)
    gazebo_cmd = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            '--server-plugin', 'libgazebo_ros_init.so',
            '--server-plugin', 'libgazebo_ros_factory.so',
            '-u',  # Start paused
        ],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )
    
    # Start Gazebo with GUI (if requested)
    gazebo_gui_cmd = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            '--server-plugin', 'libgazebo_ros_init.so', 
            '--server-plugin', 'libgazebo_ros_factory.so',
            '-u',  # Start paused
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # Static transform publisher
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'panda_link0'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Spawn robot in Gazebo (with delay to ensure Gazebo is ready)
    spawn_entity = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'panda',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.0'
                ],
                output='screen',
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
            )
        ]
    )

    # ros2_control using Gazebo
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    # Joint State Broadcaster (with delay)
    joint_state_broadcaster_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                ],
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
            )
        ]
    )

    # Panda Arm Controller (with delay)
    panda_arm_controller_spawner = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["panda_arm_controller", "-c", "/controller_manager"],
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
            )
        ]
    )

    # Panda Hand Controller (with delay)
    panda_hand_controller_spawner = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["panda_hand_controller", "-c", "/controller_manager"],
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
            )
        ]
    )

    # MoveGroup Node (with delay)
    move_group_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}
                ],
            )
        ]
    )

    # RViz (with delay)
    rviz_config_file = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "launch",
        "moveit.rviz"
    )

    rviz_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=["-d", rviz_config_file],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.planning_pipelines,
                    moveit_config.robot_description_kinematics,
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}
                ],
            )
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        gui_arg,
        gazebo_cmd,
        gazebo_gui_cmd,
        robot_state_publisher,
        static_tf,
        spawn_entity,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        panda_arm_controller_spawner,
        panda_hand_controller_spawner,
        move_group_node,
        rviz_node,
    ]) 