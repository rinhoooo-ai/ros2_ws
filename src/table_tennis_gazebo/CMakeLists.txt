cmake_minimum_required(VERSION 3.5)
project(table_tennis_gazebo)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(gz-transport13 REQUIRED)

# Generate action and service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/SpawnBall.action"
  "srv/SpawnBall.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

# Ball spawner node
add_executable(ball_spawner_node src/ball_spawner_node.cpp)
ament_target_dependencies(ball_spawner_node
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  ros_gz_interfaces
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(ball_spawner_node "${cpp_typesupport_target}" gz-transport13::gz-transport13)

# Install executables
install(TARGETS
  ball_spawner_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(
  DIRECTORY
    launch
    config
    worlds
    models
    rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts
install(PROGRAMS
  scripts/ball_spawn_client.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
