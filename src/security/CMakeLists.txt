cmake_minimum_required(VERSION 3.5)
project(security)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(oqs REQUIRED)

add_executable(secure_node src/secure_node.cpp)
ament_target_dependencies(secure_node rclcpp)
target_link_libraries(secure_node oqs::oqs)

install(TARGETS
  secure_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
