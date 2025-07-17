cmake_minimum_required(VERSION 3.8)
project(thor_manipulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables with RENAME (separate blocks required)

install(PROGRAMS
  scripts/pick_place_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pick_place_node
)

install(PROGRAMS
  scripts/pick_place_task_constructor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pick_place_task_constructor
)

install(PROGRAMS
  scripts/thor_task_constructor_demo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME thor_task_constructor_demo
)
install(PROGRAMS
  scripts/control_interface.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME control_interface
)


install(PROGRAMS
  scripts/task_coordinator_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME task_coordinator_node
)

install(PROGRAMS
  scripts/object_detector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME object_detector
)
install(PROGRAMS
  scripts/enhanced_pick_place_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME enhanced_pick_place_node
)
install(PROGRAMS
  scripts/simple_command_interface.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME simple_command_interface
)
# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)


# Install config files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install rviz config if exists
install(DIRECTORY
  rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()