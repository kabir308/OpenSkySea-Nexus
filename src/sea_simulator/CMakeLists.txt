cmake_minimum_required(VERSION 3.5)
project(sea_simulator)

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

add_executable(sea_simulator_node src/sea_simulator_node.cpp)
ament_target_dependencies(sea_simulator_node rclcpp hybrid_msgs)

install(TARGETS
  sea_simulator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
