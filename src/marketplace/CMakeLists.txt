cmake_minimum_required(VERSION 3.5)
project(marketplace)

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
  "srv/ListModules.srv"
)

add_executable(marketplace_node src/marketplace_node.cpp)
ament_target_dependencies(marketplace_node rclcpp)

install(TARGETS
  marketplace_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
