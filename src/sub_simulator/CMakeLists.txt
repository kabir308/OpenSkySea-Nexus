cmake_minimum_required(VERSION 3.5)
project(sub_simulator)

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

add_executable(sub_simulator_node src/sub_simulator_node.cpp)
ament_target_dependencies(sub_simulator_node rclcpp hybrid_msgs)

install(TARGETS
  sub_simulator_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
