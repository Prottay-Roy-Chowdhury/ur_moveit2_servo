cmake_minimum_required(VERSION 3.8)
project(ur_commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)


# Service files
# Service files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/VisualizePoses.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

# Install Python package
# ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/ur_commander)
  DEPENDENCIES builtin_interfaces geometry_msgs
)

# Install Python package
# ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/ur_commander)

# install(DIRECTORY src/ur_commander
#   DESTINATION lib/${PROJECT_NAME}
#   FILES_MATCHING PATTERN "*.py"
# )

# Install launch files.
install(DIRECTORY
  launch config 
# install(DIRECTORY src/ur_commander
#   DESTINATION lib/${PROJECT_NAME}
#   FILES_MATCHING PATTERN "*.py"
# )

# Install launch files.
install(DIRECTORY
  launch config 
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  src/ur_commander/visualize_pose_srv.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/aruco_detection.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/camera_static.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

