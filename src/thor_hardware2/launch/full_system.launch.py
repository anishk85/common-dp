FILE: launch/full_system.launch.py
================================================================================
#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package directory
    pkg_share = FindPackageShare('thor_hardware2').find('thor_hardware2')
    
    # Launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',  
        default_value='/dev/ttyACM0',
        description='Arduino serial port'
    )
    
    enable_cameras_arg = DeclareLaunchArgument(
        'enable_cameras',
        default_value='true', 
        description='Enable camera nodes'
    )
    
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

    # Hardware launch
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_share,
                'launch',
                'hardware.launch.py'
            ])
        ]),
        launch_arguments={
            'serial_port': LaunchConfiguration('serial_port')
        }.items()
    )

    # Camera launch (delayed to allow hardware to initialize)
    cameras_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        pkg_share, 
                        'launch',
                        'cameras.launch.py'
                    ])
                ]),
                condition=LaunchConfiguration('enable_cameras')
            )
        ]
    )

    # MoveIt launch (if available)
    moveit_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('thor_moveit_config'),
                        'launch', 
                        'move_group.launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': 'false',
                    'allow_trajectory_execution': 'true',
                    'publish_monitored_planning_scene': 'true'
                }.items()
            )
        ]
    )

    # RViz
    rviz_config = PathJoinSubstitution([
        pkg_share,
        'config',
        'thor_hardware.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=LaunchConfiguration('enable_rviz')
    )

    return LaunchDescription([
        serial_port_arg,
        enable_cameras_arg,
        enable_rviz_arg,
        hardware_launch,
        cameras_launch,
        moveit_launch,
        TimerAction(period=8.0, actions=[rviz_node])
    ])
