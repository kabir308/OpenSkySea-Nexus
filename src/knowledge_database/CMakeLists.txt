cmake_minimum_required(VERSION 3.5)
project(knowledge_database)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sqlite3 REQUIRED)

add_executable(knowledge_database_node src/knowledge_database_node.cpp)
ament_target_dependencies(knowledge_database_node rclcpp)
target_link_libraries(knowledge_database_node sqlite3)

install(TARGETS
  knowledge_database_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
