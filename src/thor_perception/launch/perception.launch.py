import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Camera configuration
    camera_config = os.path.join(
        get_package_share_directory('thor_perception'),
        'config',
        'camera_config.yaml'
    )
    
    # Perception parameters
    perception_params = os.path.join(
        get_package_share_directory('thor_perception'),
        'config',
        'perception_params.yaml'
    )
    
    # Perception node
    perception_node = Node(
        package='thor_perception',
        executable='perception_node.py',
        name='perception_node',
        output='screen',
        parameters=[
            perception_params,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Pick and place controller
    pick_place_node = Node(
        package='thor_perception',
        executable='pick_place_controller.py',
        name='pick_place_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Static transform for camera
    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'camera_optical_frame', 'camera_link']
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        perception_node,
        pick_place_node,
        camera_tf_node
    ])