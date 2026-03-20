cmake_minimum_required(VERSION 3.8)
project(rcm_gripper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rcm_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

add_library(gripper_base SHARED src/gripper_base.cpp)
add_library(generic_end_effector_base SHARED src/generic_end_effector_base.cpp)

add_executable(gripper_controller src/gripper_controller.cpp)

ament_target_dependencies(
  gripper_base
  std_srvs
  rclcpp
)

ament_target_dependencies(
  generic_end_effector_base
  rcm_msgs
  rclcpp
  rclcpp_action
  std_srvs
)

ament_export_dependencies(
  rcm_msgs
  std_srvs
)

target_link_libraries(gripper_controller gripper_base generic_end_effector_base)

ament_target_dependencies(
  gripper_controller
  pluginlib
  rclcpp
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  gripper_base
  generic_end_effector_base
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  gripper_controller
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(
  include
)

ament_export_libraries(
  gripper_base
  generic_end_effector_base
)

ament_package()

