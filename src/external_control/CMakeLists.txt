cmake_minimum_required(VERSION 3.5)
project(external_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(httplib REQUIRED)

add_executable(external_control_node src/external_control_node.cpp)
ament_target_dependencies(external_control_node rclcpp)
target_link_libraries(external_control_node httplib::httplib)

install(TARGETS
  external_control_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
