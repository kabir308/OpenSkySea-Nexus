cmake_minimum_required(VERSION 3.5)
project(distributed_learning)

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
  "msg/LearningModel.msg"
)

add_executable(distributed_learning_node src/distributed_learning_node.cpp)
ament_target_dependencies(distributed_learning_node rclcpp)

install(TARGETS
  distributed_learning_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
