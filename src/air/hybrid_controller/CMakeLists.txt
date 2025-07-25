cmake_minimum_required(VERSION 3.5)
project(hybrid_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hybrid_msgs REQUIRED)

add_executable(hybrid_controller_node src/hybrid_controller_node.cpp)
ament_target_dependencies(hybrid_controller_node rclcpp hybrid_msgs)

install(TARGETS
  hybrid_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
