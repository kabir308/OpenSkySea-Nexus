cmake_minimum_required(VERSION 3.5)
project(vision_hub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(perception_msgs REQUIRED)

add_executable(vision_hub_node src/vision_hub_node.cpp)
ament_target_dependencies(vision_hub_node rclcpp sensor_msgs perception_msgs)

install(TARGETS
  vision_hub_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
