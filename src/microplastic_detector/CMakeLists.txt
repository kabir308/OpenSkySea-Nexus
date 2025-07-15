cmake_minimum_required(VERSION 3.5)
project(microplastic_detector)

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

add_executable(microplastic_detector_node src/microplastic_detector_node.cpp)
ament_target_dependencies(microplastic_detector_node rclcpp sensor_msgs)

install(TARGETS
  microplastic_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
