cmake_minimum_required(VERSION 3.8)
project(rcm_clients)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rcm_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

add_library(get_joint_states_client src/get_joint_states_client.cpp)

ament_target_dependencies(
  get_joint_states_client
  rclcpp
  rcm_msgs
)

add_library(joint_movement_client src/joint_movement_client.cpp)

ament_target_dependencies(
  joint_movement_client
  rclcpp
  rclcpp_action
  rcm_msgs
)

add_library(get_pose_client src/get_pose_client.cpp)

ament_target_dependencies(
  get_pose_client
  geometry_msgs
  rclcpp
  rcm_msgs
)

add_library(cartesian_movement_client src/cartesian_movement_client.cpp)

ament_target_dependencies(
  cartesian_movement_client
  geometry_msgs
  rclcpp
  rclcpp_action
  rcm_msgs
)

add_library(pose_movement_client src/pose_movement_client.cpp)

ament_target_dependencies(
  pose_movement_client
  geometry_msgs
  rclcpp
  rclcpp_action
  rcm_msgs
)

add_library(control_end_effector_client src/control_end_effector_client.cpp)

ament_target_dependencies(
  control_end_effector_client
  rclcpp
  rclcpp_action
  rcm_msgs
)

add_library(pick_and_place_client src/pick_and_place_client.cpp)

ament_target_dependencies(
  pick_and_place_client
  rclcpp
  rclcpp_action
  rcm_msgs
)


ament_target_dependencies(
  rclcpp
  std_srvs
)

add_library(rcm_manager_client src/rcm_manager_client.cpp)

ament_target_dependencies(
  rcm_manager_client
  rclcpp
  std_srvs
  rcm_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(rcm_clientsTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(get_joint_states_client)
ament_export_libraries(joint_movement_client)
ament_export_libraries(get_pose_client)
ament_export_libraries(cartesian_movement_client)
ament_export_libraries(pose_movement_client)
ament_export_libraries(control_end_effector_client)
ament_export_libraries(pick_and_place_client)
ament_export_libraries(rcm_manager_client)

ament_export_dependencies(
  geometry_msgs
  rclcpp
  rclcpp_action
  rcm_msgs
  std_srvs)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  get_joint_states_client
  joint_movement_client
  get_pose_client
  cartesian_movement_client
  pose_movement_client
  control_end_effector_client
  pick_and_place_client
  rcm_manager_client
  EXPORT rcm_clientsTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

ament_package()
