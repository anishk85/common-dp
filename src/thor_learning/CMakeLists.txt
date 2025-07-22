cmake_minimum_required(VERSION 3.8)
project(thor_learning)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install the Python executable
install(
  DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()