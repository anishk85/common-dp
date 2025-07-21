cmake_minimum_required(VERSION 3.8)
project(thor_teleop)

# Find all necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


# C++ executable for pose control
add_executable(teleop_pose_control src/teleop_pose_control.cpp)
ament_target_dependencies(teleop_pose_control
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  moveit_msgs
)
install(TARGETS
  teleop_pose_control
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python executables
# You can uncomment this if you also have the Python scripts
# install(
#   PROGRAMS scripts/teleop_joint_control.py
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()