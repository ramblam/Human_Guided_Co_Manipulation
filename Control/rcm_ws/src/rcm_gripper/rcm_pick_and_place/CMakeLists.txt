cmake_minimum_required(VERSION 3.8)
project(rcm_pick_and_place)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcm_msgs REQUIRED)
find_package(rcm_clients REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)
add_executable(pick_and_place_server src/pick_and_place_server.cpp)
add_executable(handover_server src/handover_server.cpp)

add_library(pap_action STATIC src/pap_action.cpp)
add_library(pap_controller STATIC src/pap_controller.cpp)
add_library(pap_services STATIC src/pap_services.cpp)
add_library(handover_controller STATIC src/handover_controller.cpp)

ament_target_dependencies(
  pap_action
  geometry_msgs
  rcm_msgs
  rclcpp
)
 
target_link_libraries(
  pap_action
  pap_controller
)

ament_target_dependencies(
  pap_controller
  rcm_clients
  geometry_msgs
  rcm_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
)

target_link_libraries(
  pap_services
  pap_controller
)

ament_target_dependencies(
  pap_services
  geometry_msgs
  rcm_msgs
  rclcpp
)

target_link_libraries(
  pick_and_place_server
  pap_services
  pap_action
)

ament_target_dependencies(
  pick_and_place_server
  rcm_clients
  rclcpp
  tf2
)

ament_target_dependencies(
  handover_controller
  rcm_clients
  geometry_msgs
  rclcpp
  tf2
)

ament_target_dependencies(
  handover_server
  rclcpp
)

target_link_libraries(
  handover_server
  handover_controller
)

install(TARGETS
  pap_action
  pap_services
  pap_controller
  handover_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  pick_and_place_server
  handover_server
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
#install(DIRECTORY
#launch
#DESTINATION share/${PROJECT_NAME}/
#)

ament_package()
