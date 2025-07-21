cmake_minimum_required(VERSION 3.8)
project(thor_teleop)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(moveit_msgs REQUIRED)

# Install Python executables
install(
  PROGRAMS scripts/teleop_joint_control.py
  scripts/teleop_pose_control.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()