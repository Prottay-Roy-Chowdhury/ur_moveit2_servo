cmake_minimum_required(VERSION 3.8)
project(ur_commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies (only once per package)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

# Service files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/VisualizePoses.srv"
  "srv/GetTransform.srv"
  "srv/SavePointCloud.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

# Install Python package manually
ament_python_install_package(commander_py)

# Install launch and config files
install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/camera_info_publisher.py
  scripts/image_publisher.py
  scripts/tf_service_server.py
  scripts/tf_service_client.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  commander_py/commander.py
  commander_py/commander_utils.py
  commander_py/commander_viz.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()