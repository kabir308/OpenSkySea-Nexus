cmake_minimum_required(VERSION 3.5)
project(simple_ui)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nexus_communication_protocol REQUIRED)

add_executable(simple_ui_node src/simple_ui_node.cpp)
ament_target_dependencies(simple_ui_node rclcpp nexus_communication_protocol)

install(TARGETS
  simple_ui_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
