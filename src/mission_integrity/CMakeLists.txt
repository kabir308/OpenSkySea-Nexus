cmake_minimum_required(VERSION 3.5)
project(mission_integrity)

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
  "msg/MissionLog.msg"
  DEPENDENCIES std_msgs
)

add_executable(mission_logger_node src/mission_logger_node.cpp)
ament_target_dependencies(mission_logger_node rclcpp)

install(TARGETS
  mission_logger_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
