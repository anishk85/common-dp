cmake_minimum_required(VERSION 3.8)
project(thor_hardware)

# Find all necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ros2_control_od REQUIRED)
find_package(libserial-dev REQUIRED)

# Build the C++ hardware interface as a shared library
add_library(thor_arm_hardware_interface SHARED
  src/thor_arm_hardware_interface.cpp
)
ament_target_dependencies(thor_arm_hardware_interface
  rclcpp
  hardware_interface
  pluginlib
  ros2_control_od
  libserial-dev
)

# Register the hardware interface as a plugin for ros2_control
pluginlib_export_plugin_description_file(hardware_interface src/thor_hardware.xml)

# Install the compiled library
install(TARGETS
  thor_arm_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers (good practice)
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch, config, and URDF files
install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)
install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)
install(
    DIRECTORY urdf
    DESTINATION share/${PROJECT_NAME}
)

ament_package()