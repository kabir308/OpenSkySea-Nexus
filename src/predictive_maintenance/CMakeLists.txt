cmake_minimum_required(VERSION 3.5)
project(predictive_maintenance)

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
  "msg/ComponentState.msg"
)

add_executable(predictive_maintenance_node src/predictive_maintenance_node.cpp)
ament_target_dependencies(predictive_maintenance_node rclcpp)

install(TARGETS
  predictive_maintenance_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
