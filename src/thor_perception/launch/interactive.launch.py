#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Launch arguments - FIXED: Match complete_demo pattern
    is_sim_arg = DeclareLaunchArgument(
        'is_sim',
        default_value='True',  # FIXED: Capital T like complete_demo
        description='Use simulation time'
    )
    
    # Launch configurations
    is_sim = LaunchConfiguration('is_sim')
    
    # 1. Gazebo launch - FIXED: Use same pattern as complete_demo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("thor_urdf"),
                "launch", 
                "gazebo.launch.py"
            )
        )
    )
    
    # 2. Controller launch - FIXED: Add controller launch like complete_demo
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
    
    # 3. MoveIt launch - FIXED: Optional MoveIt like complete_demo
    try:
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
        moveit_available = True
    except Exception:
        print("⚠️ MoveIt not available - skipping")
        moveit_available = False
    
    # # 4. Enhanced perception launch - FIXED: Match complete_demo pattern
    # perception_launch = TimerAction(
    #     period=5.0,
    #     actions=[
    #         Node(
    #             package='thor_perception',
    #             executable='perception_node.py',  # FIXED: Use .py extension like complete_demo
    #             name='perception_node',
    #             output='screen',
    #             parameters=[{'use_sim_time': is_sim}]
    #         )
    #     ]
    # )
    
    # 5. Auto object spawner - FIXED: Use same pattern as complete_demo
    auto_spawner = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='thor_perception',
                executable='auto_object_spawner.py',  # FIXED: Add .py extension
                name='auto_object_spawner',
                output='screen',
                parameters=[{'use_sim_time': is_sim}]
            )
        ]
    )
    
    # 6. Interactive controller - FIXED: Match complete_demo pattern
    interactive_controller = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='thor_perception',
                executable='interactive_pick_place.py',  # FIXED: Add .py extension
                name='interactive_pick_place_controller',
                output='screen',
                parameters=[{'use_sim_time': is_sim}]
            )
        ]
    )
    
    # Build launch description - FIXED: Include all components properly
    launch_actions = [
        is_sim_arg,
        gazebo_launch,
        controller_launch,
        # perception_launch,
        auto_spawner,
        interactive_controller,
    ]
    
    # Add MoveIt if available
    if moveit_available:
        launch_actions.insert(3, moveit_launch)  # Insert after controller_launch
    
    return LaunchDescription(launch_actions)