import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen"
    )

    ps5_teleop_node = Node(
        package='thor_teleop',  # Replace with your actual package name
        executable='teleop_ps5_control_joy.py',
        name='teleop_ps5_control_joy',
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        joy_node,
        ps5_teleop_node,
    ])