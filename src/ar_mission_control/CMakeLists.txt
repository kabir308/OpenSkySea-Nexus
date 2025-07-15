cmake_minimum_required(VERSION 3.5)
project(ar_mission_control)

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
  "msg/GestureCommand.msg"
  "msg/VoiceCommand.msg"
  "msg/VisualizationData.msg"
  DEPENDENCIES std_msgs
)

add_executable(ar_mission_control_node src/ar_mission_control_node.cpp)
ament_target_dependencies(ar_mission_control_node rclcpp)

install(TARGETS
  ar_mission_control_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
