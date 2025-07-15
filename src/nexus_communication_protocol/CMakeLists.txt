cmake_minimum_required(VERSION 3.5)
project(nexus_communication_protocol)

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
find_package(pluginlib REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GoToWaypoint.msg"
  "msg/SetVelocity.msg"
  "msg/FollowTrajectory.msg"
  "msg/InspectZone.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

add_library(mavros_translator src/mavros_translator.cpp)
target_link_libraries(mavros_translator pluginlib)

add_library(pypilot_translator src/pypilot_translator.cpp)
target_link_libraries(pypilot_translator pluginlib)

add_library(custom_translator src/custom_translator.cpp)
target_link_libraries(custom_translator pluginlib)

pluginlib_export_plugin_description_file(nexus_communication_protocol translator_plugins.xml)

add_executable(ncp_translator_node src/ncp_translator_node.cpp)
ament_target_dependencies(ncp_translator_node rclcpp mavros_msgs)

install(TARGETS
  ncp_translator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  mavros_translator
  pypilot_translator
  custom_translator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
