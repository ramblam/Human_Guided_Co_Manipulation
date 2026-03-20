cmake_minimum_required(VERSION 3.8)
project(rcm_gripper_franka_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcm_gripper REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)

# Generate plugins
pluginlib_export_plugin_description_file(rcm_gripper plugins.xml)

include_directories(include)

add_library(franka_hand src/franka_hand.cpp)
target_compile_features(franka_hand PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(franka_hand PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  franka_hand
  franka_msgs
  rclcpp
  rclcpp_action
  pluginlib
  rcm_gripper
  std_srvs
)

add_library(franka_end_effector src/franka_end_effector.cpp)
target_compile_features(franka_end_effector PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(franka_end_effector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  franka_end_effector
  franka_msgs
  rclcpp
  rclcpp_action
  pluginlib
  rcm_gripper
  std_srvs
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
# target_compile_definitions(dummy_gripper PRIVATE "RCM_GRIPPER_PLUGINS_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS franka_hand franka_end_effector
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}/
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

ament_export_include_directories(
  include
)
#ament_export_libraries(
#  franka_hand franka_end_effector
#)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
