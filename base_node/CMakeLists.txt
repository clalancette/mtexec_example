cmake_minimum_required(VERSION 3.5)
project(base_node)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(base_node src/base_node.cpp)
ament_target_dependencies(base_node
  nav_msgs
  rclcpp
)
install(TARGETS
  base_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
