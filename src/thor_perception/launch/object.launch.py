# launch/spawn_objects.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    world_file = LaunchConfiguration('world_file')
    
    # Gazebo with custom world
    gazebo_world = os.path.join(
        get_package_share_directory('thor_perception'),
        'worlds',
        'pick_place_world.sdf'
    )
    
    # Spawn objects service call
    spawn_objects_cmd = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn_random_objects', 'std_srvs/srv/Empty'],
        name='spawn_objects'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_file',
            default_value=gazebo_world,
            description='Path to world file'
        ),
        spawn_objects_cmd
    ])