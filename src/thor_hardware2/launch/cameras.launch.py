FILE: launch/cameras.launch.py
================================================================================
#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    camera_config_arg = DeclareLaunchArgument(
        'camera_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('thor_hardware2'),
            'config',
            'camera_config.yaml'
        ]),
        description='Camera configuration file'
    )

    # Pi Camera 1 - Workspace Overview
    pi_camera_1 = Node(
        package='thor_hardware2',
        executable='camera_node.py',
        name='pi_camera_1',
        output='screen',
        parameters=[{
            'camera_id': 0,
            'camera_name': 'workspace_overview',
            'camera_type': 'pi_camera',
            'frame_id': 'camera_1_frame',
            'width': 1920,
            'height': 1080, 
            'fps': 30,
            'auto_focus': True
        }],
        remappings=[
            ('image_raw', 'camera_1/image_raw'),
            ('camera_info', 'camera_1/camera_info')
        ]
    )

    # Pi Camera 2 - Close-up View
    pi_camera_2 = Node(
        package='thor_hardware2',
        executable='camera_node.py', 
        name='pi_camera_2',
        output='screen',
        parameters=[{
            'camera_id': 1,
            'camera_name': 'close_up_view',
            'camera_type': 'pi_camera',
            'frame_id': 'camera_2_frame',
            'width': 1920,
            'height': 1080,
            'fps': 30,
            'auto_focus': True
        }],
        remappings=[
            ('image_raw', 'camera_2/image_raw'),
            ('camera_info', 'camera_2/camera_info')
        ]
    )

    # USB Mobile Camera - Handheld/Additional View
    usb_camera = Node(
        package='thor_hardware2',
        executable='camera_node.py',
        name='usb_mobile_camera',
        output='screen', 
        parameters=[{
            'camera_id': '/dev/video0',
            'camera_name': 'mobile_camera',
            'camera_type': 'usb_camera',
            'frame_id': 'mobile_camera_frame',
            'width': 1280,
            'height': 720,
            'fps': 30,
            'auto_focus': False
        }],
        remappings=[
            ('image_raw', 'mobile_camera/image_raw'),
            ('camera_info', 'mobile_camera/camera_info')
        ]
    )

    # Camera Calibration Node (optional)
    camera_calibration = Node(
        package='camera_calibration',
        executable='cameracalibrator.py',
        name='camera_calibration',
        output='screen',
        arguments=[
            '--size', '8x6',
            '--square', '0.025',
            'image:=/camera_1/image_raw',
            'camera:=/camera_1'
        ],
        condition='false'  # Set to 'true' when calibrating
    )

    return LaunchDescription([
        camera_config_arg,
        pi_camera_1,
        pi_camera_2, 
        usb_camera,
        camera_calibration
    ])
