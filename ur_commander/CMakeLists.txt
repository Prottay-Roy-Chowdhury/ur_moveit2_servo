cmake_minimum_required(VERSION 3.8)
project(ur_commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)  # <-- Add this line
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces first
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/VisualizePoses.srv"
  DEPENDENCIES geometry_msgs  # Explicitly list dependencies
)

# Export ALL dependencies used in interfaces
ament_export_dependencies(
  rosidl_default_runtime
  geometry_msgs
  std_msgs
  cv_bridge
  rosidl_default_generators
)

ament_python_install_package(
commander_py
)

# Install directories and scripts
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  commander_py/commander_viz.py
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()