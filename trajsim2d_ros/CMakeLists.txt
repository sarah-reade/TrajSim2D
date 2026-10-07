cmake_minimum_required(VERSION 3.8)
project(trajsim2d_ros)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Simulation.action"
)

# rosidl generates the Python package directory with the same name as this
# package. Install the node module into that directory instead of invoking
# ament_python_install_package, whose package-name filter would exclude it.
install(FILES
  ${PROJECT_NAME}/node.py
  ${PROJECT_NAME}/direct_trajectory_test_node.py
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/trajsim2d_node
  scripts/direct_trajectory_test_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
