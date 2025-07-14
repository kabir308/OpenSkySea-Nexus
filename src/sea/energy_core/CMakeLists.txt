cmake_minimum_required(VERSION 3.5)
project(energy_core)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(energy_msgs REQUIRED)

add_executable(energy_core_node src/energy_core_node.cpp)
ament_target_dependencies(energy_core_node rclcpp energy_msgs)

install(TARGETS
  energy_core_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
