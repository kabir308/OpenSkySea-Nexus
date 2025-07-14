cmake_minimum_required(VERSION 3.5)
project(multi_channel_stack)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(multi_channel_msgs REQUIRED)

add_executable(multi_channel_stack_node src/multi_channel_stack_node.cpp)
ament_target_dependencies(multi_channel_stack_node rclcpp multi_channel_msgs)

install(TARGETS
  multi_channel_stack_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
