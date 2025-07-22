# File: thor_hardware/launch/hardware.launch.py
# ========================================================================
# This is the main launch file to start the physical robot.
#
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
import xacro

def generate_launch_description():
    # Get the path to the URDF file
    robot_description_path = os.path.join(
        get_package_share_directory('thor_urdf'),
        'urdf',
        'thor_arm.urdf.xacro'
    )
    
    # Process the URDF file
    robot_description_config = xacro.process_file(robot_description_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Get the path to the controllers config file
    controllers_config_path = os.path.join(
        get_package_share_directory('thor_controllers'),
        'launch',
        'controller.launch.py'
    )
    launch_arguments={"is_sim": "True"}.items()


    # --- Nodes to Launch ---

    # 1. robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 2. ros2_control_node
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_config_path],
        output='screen',
    )

    # 3. joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    # 4. joint_trajectory_controller
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
    )
    
    # 5. MoveIt's move_group node
    # This assumes you have a launch file in your MoveIt config package that starts move_group
    # For now, we'll use a generic one. You should replace this with your actual MoveIt launch.
    run_move_group_node = ExecuteProcess(
        cmd=['ros2', 'launch', 'thor_moveit_config', 'move_group.launch.py'],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        run_move_group_node
    ])

    # this file needs to be updated to include the moveit launch file






#     # ========================================================================
# # File: thor_hardware/CMakeLists.txt
# # ========================================================================
# # This file is updated to install all necessary launch files.
# #
# cmake_minimum_required(VERSION 3.8)
# project(thor_hardware)

# # Find all necessary packages
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_action REQUIRED)
# find_package(hardware_interface REQUIRED)
# find_package(pluginlib REQUIRED)
# find_package(ros2_control_od REQUIRED)
# find_package(libserial-dev REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)
# find_package(moveit_msgs REQUIRED)

# # Build the C++ hardware interface
# add_library(thor_arm_hardware_interface SHARED
#   src/thor_arm_hardware_interface.cpp
# )
# ament_target_dependencies(thor_arm_hardware_interface
#   rclcpp rclcpp_action hardware_interface pluginlib ros2_control_od
#   libserial-dev std_msgs geometry_msgs tf2_ros tf2_geometry_msgs moveit_msgs
# )
# pluginlib_export_plugin_description_file(hardware_interface src/thor_hardware.xml)

# # Install the compiled library and headers
# install(TARGETS thor_arm_hardware_interface
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )
# install(DIRECTORY include/ DESTINATION include)

# # Install all config, urdf, and launch files
# install(
#     DIRECTORY config urdf launch
#     DESTINATION share/${PROJECT_NAME}
# )

# ament_package()
# ```python
# # ========================================================================
# # File: thor_hardware/launch/hardware.launch.py
# # ========================================================================
# # This is the NEW top-level launch file for the physical robot.
# # It follows the modular structure you provided.
# #
# import os
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration
# from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():
#     # Declare the launch argument
#     declared_arguments = []
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "is_sim",
#             default_value="false",
#             description="Set to true for simulation, false for physical hardware.",
#         )
#     )

#     # Initialize the argument
#     is_sim = LaunchConfiguration("is_sim")

#     # 1. Include the controllers launch file
#     # This starts the ros2_control_node and spawns the controllers
#     controller_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(
#                 get_package_share_directory("thor_hardware"),
#                 "launch",
#                 "controllers.launch.py",
#             )
#         ),
#         launch_arguments={"is_sim": is_sim}.items(),
#     )

#     # 2. Include the MoveIt launch file
#     # This starts the move_group node and other MoveIt components
#     moveit_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(
#                 get_package_share_directory("thor_moveit_config"),
#                 "launch",
#                 "moveit.launch.py", # Using the top-level MoveIt launch is better
#             )
#         ),
#         launch_arguments={"use_sim_time": is_sim}.items(),
#     )

#     return LaunchDescription(declared_arguments + [
#         controller_launch,
#         moveit_launch,
#     ])
# ```python
# # ========================================================================
# # File: thor_hardware/launch/controllers.launch.py
# # ========================================================================
# # This new file specifically handles starting the robot driver and controllers.
# # It is included by the main hardware.launch.py file.
# #
# import os
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration, Command
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():
#     # Declare the launch argument
#     declared_arguments = []
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "is_sim",
#             default_value="false",
#             description="Passed from the parent launch file.",
#         )
#     )
    
#     # Initialize the argument
#     is_sim = LaunchConfiguration("is_sim")

#     # Get the URDF
#     robot_description_content = Command(
#         [
#             "xacro ",
#             os.path.join(
#                 get_package_share_directory("thor_urdf"), "urdf", "thor_arm.urdf.xacro"
#             ),
#             " is_sim:=", is_sim,
#         ]
#     )
#     robot_description = {"robot_description": robot_description_content}

#     # Get the controllers config file
#     controllers_config_path = os.path.join(
#         get_package_share_directory('thor_hardware'), 'config', 'controllers.yaml'
#     )

#     # Define the nodes
#     robot_state_publisher_node = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         output='screen',
#         parameters=[robot_description]
#     )

#     ros2_control_node = Node(
#         package='controller_manager',
#         executable='ros2_control_node',
#         parameters=[robot_description, controllers_config_path],
#         output='screen',
#     )

#     joint_state_broadcaster_spawner = Node(
#         package='controller_manager',
#         executable='spawner',
#         arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
#     )

#     joint_trajectory_controller_spawner = Node(
#         package='controller_manager',
#         executable='spawner',
#         arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
#     )

#     return LaunchDescription(declared_arguments + [
#         robot_state_publisher_node,
#         ros2_control_node,
#         joint_state_broadcaster_spawner,
#         joint_trajectory_controller_spawner,
#     ])















