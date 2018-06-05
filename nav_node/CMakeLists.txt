cmake_minimum_required(VERSION 3.5)
project(nav_node)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(gtg_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(nav_node src/nav_node.cpp)
ament_target_dependencies(nav_node
  gtg_msgs
  nav_msgs
  rclcpp
)
install(TARGETS
  nav_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
