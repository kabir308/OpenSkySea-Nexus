cmake_minimum_required(VERSION 3.5)
project(predictive_energy_manager)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(energy_prediction_msgs REQUIRED)

add_executable(predictive_energy_manager_node src/predictive_energy_manager_node.cpp)
ament_target_dependencies(predictive_energy_manager_node rclcpp energy_prediction_msgs energy_msgs)

install(TARGETS
  predictive_energy_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
