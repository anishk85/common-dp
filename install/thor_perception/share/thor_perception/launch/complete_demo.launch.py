import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
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
    
    # Original thor launch
    thor_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("thor_urdf"),
            "launch",
            "gazebo.launch.py"
        )
    )
    
    # Controller launch
    controller_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                os.path.join(
                    get_package_share_directory("thor_controller"),
                    "launch",
                    "controller.launch.py"
                ),
                launch_arguments={"is_sim": is_sim}.items()
            )
        ]
    )
    
    # MoveIt launch
    moveit_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                os.path.join(
                    get_package_share_directory("thor_moveit"),
                    "launch",
                    "moveit.launch.py"
                ),
                launch_arguments={"is_sim": is_sim}.items()
            )
        ]
    )
    
    # Enhanced perception launch
    perception_launch = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='thor_perception',
                executable='perception_node.py',
                name='perception_node',
                output='screen',
                parameters=[{'use_sim_time': is_sim}]
            )
        ]
    )
    
    # Object spawner node
    object_spawner_node = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='thor_perception',
                executable='object_spawner.py',
                name='object_spawner',
                output='screen',
                parameters=[{'use_sim_time': is_sim}]
            )
        ]
    )
    
    # Auto-spawn objects
    auto_spawn_objects = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='thor_perception',
                executable='auto_spawn_trigger.py',
                name='auto_spawn_trigger',
                output='screen',
                parameters=[{'use_sim_time': is_sim}]
            )
        ]
    )
    
    # SINGLE coordinated pick and place controller
    pick_place_node = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='thor_perception',
                executable='pick_place_controller.py',
                name='pick_place_controller',
                output='screen',
                parameters=[{'use_sim_time': is_sim}]
            )
        ]
    )
    
    return LaunchDescription([
        is_sim_arg,
        thor_launch,
        controller_launch,
        moveit_launch,
        perception_launch,
        object_spawner_node,
        auto_spawn_objects,
        pick_place_node
    ])