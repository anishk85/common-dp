import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
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
        executable='teleop_ps5_control_pose.py',
        name='teleop_ps5_control_pose',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        joy_node,
        ps5_teleop_node,
    ])