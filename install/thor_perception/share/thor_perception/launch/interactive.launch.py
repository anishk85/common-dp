#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Launch arguments
    is_sim_arg = DeclareLaunchArgument(
        'is_sim',
        default_value='True',
        description='Use simulation time'
    )
    
    # Launch configurations
    is_sim = LaunchConfiguration('is_sim')
    
    # 1. Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("thor_urdf"),
                "launch", 
                "gazebo.launch.py"
            )
        )
    )
    
    # 2. Controller launch
    controller_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("thor_controller"),
                        "launch",
                        "controller.launch.py"
                    )
                ),
                launch_arguments={"is_sim": is_sim}.items()
            )
        ]
    )
    
    # 3. MoveIt launch - REQUIRED for proper motion planning
    moveit_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("thor_moveit"),
                        "launch",
                        "moveit.launch.py"
                    )
                ),
                launch_arguments={"is_sim": is_sim}.items()
            )
        ]
    )
    
    # 4. Auto object spawner
    auto_spawner = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='thor_perception',
                executable='auto_object_spawner.py',
                name='auto_object_spawner',
                output='screen',
                parameters=[{'use_sim_time': is_sim}]
            )
        ]
    )
    
    # 5. FIXED: Interactive controller using MoveIt
    interactive_controller = TimerAction(
        period=15.0,  # Wait longer for MoveIt to be ready
        actions=[
            Node(
                package='thor_perception',
                executable='interactive_pick_place.py',
                name='interactive_pick_place_controller',
                output='screen',
                parameters=[{'use_sim_time': is_sim}]
            )
        ]
    )
    
    return LaunchDescription([
        is_sim_arg,
        gazebo_launch,
        controller_launch,
        moveit_launch,  # REQUIRED: MoveIt for motion planning
        auto_spawner,
        interactive_controller,
    ])