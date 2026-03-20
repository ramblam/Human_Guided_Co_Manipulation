cmake_minimum_required(VERSION 3.8)
project(rcm_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcm_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(controller_manager_msgs REQUIRED)

include_directories(
  include
)

add_library(manager_base SHARED src/manager_base.cpp)

add_executable(manager_node src/manager_node.cpp)

ament_target_dependencies(
  manager_base
  PUBLIC
  controller_manager_msgs
  geometry_msgs
  rcm_msgs
  rclcpp
  std_srvs
)

target_link_libraries(manager_node manager_base)

ament_target_dependencies(
  manager_node
  std_srvs
  pluginlib
  rclcpp
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS manager_base
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

install(
  TARGETS manager_node
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(
  controller_manager_msgs
  geometry_msgs  
  rcm_msgs
  std_srvs
  rclcpp
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()

