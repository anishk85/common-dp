FILE: launch/hardware.launch.py
================================================================================
#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for Arduino communication'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate', 
        default_value='115200',
        description='Serial baud rate'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Get package directories
    pkg_share = FindPackageShare('thor_hardware2').find('thor_hardware2')
    
    # Robot description
    robot_description = PathJoinSubstitution([
        FindPackageShare('thor_urdf'),
        'urdf',
        'thor.urdf.xacro'
    ])
    
    # Hardware configuration
    hardware_config = PathJoinSubstitution([
        pkg_share,
        'config',
        'thor_hardware.yaml'
    ])
    
    # Controllers configuration  
    controllers_config = PathJoinSubstitution([
        pkg_share,
        'config', 
        'thor_controllers.yaml'
    ])

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Controller Manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[
            hardware_config,
            controllers_config,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate')
            }
        ]
    )

    # Joint State Broadcaster
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Arm Controller
    arm_controller = Node(
        package='controller_manager', 
        executable='spawner',
        arguments=['thor_arm_controller'],
        output='screen'
    )

    # Gripper Controller  
    gripper_controller = Node(
        package='controller_manager',
        executable='spawner', 
        arguments=['thor_gripper_controller'],
        output='screen'
    )

    # System Monitor
    system_monitor = Node(
        package='thor_hardware2',
        executable='system_monitor.py',
        name='system_monitor',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'monitor_frequency': 10.0
        }]
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg, 
        use_sim_time_arg,
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster,
        arm_controller,
        gripper_controller,
        system_monitor
    ])
