cmake_minimum_required(VERSION 3.8)
project(pick_n_place_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =============================
# Find dependencies
# =============================
find_package(ament_cmake REQUIRED)

# =============================
# Install folders (no nodes)
# =============================
install(
  DIRECTORY
    launch
    worlds
    models
    config
    rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()