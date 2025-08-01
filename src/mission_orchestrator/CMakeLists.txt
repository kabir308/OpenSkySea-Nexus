cmake_minimum_required(VERSION 3.5)
project(mission_orchestrator)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Mission.msg"
  "msg/Objective.msg"
  "msg/Constraint.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

add_executable(mission_generator_node src/mission_generator_node.cpp)
ament_target_dependencies(mission_generator_node rclcpp)

install(TARGETS
  mission_generator_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
