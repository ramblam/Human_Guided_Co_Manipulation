cmake_minimum_required(VERSION 3.8)
project(co_manipulation_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcm_msgs REQUIRED)
find_package(rcm_clients REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(fmt REQUIRED)

include_directories(include)

add_library(voice_command_client src/voice_command_action_client.cpp)
add_library(voice_command_action STATIC src/voice_command_action.cpp)
add_library(own_controller STATIC src/own_controller.cpp)
add_executable(voice_command_test src/voice_command_test.cpp)
add_executable(voice_command_test_v2 src/voice_command_test_v2.cpp)
add_executable(voice_command_test_listening src/voice_command_test_listening.cpp)
add_library(tts_action_client src/tts_action_client.cpp)

ament_target_dependencies(
  voice_command_client
  rclcpp
  rclcpp_action
  rcm_msgs
)

ament_target_dependencies(
  tts_action_client
  rclcpp
  rclcpp_action
  rcm_msgs
)

ament_target_dependencies(
  voice_command_action
  geometry_msgs
  rcm_msgs
  rclcpp
  rclcpp_action
  fmt
  std_srvs
)

ament_target_dependencies(
  voice_command_test
  rclcpp
  rcm_clients
  geometry_msgs
  tf2
  rcm_clients
  std_msgs
  fmt
)

ament_target_dependencies(
  voice_command_test_v2
  rclcpp
  rcm_clients
  geometry_msgs
  tf2
  rcm_clients
  std_msgs
  fmt
)

ament_target_dependencies(
  voice_command_test_listening
  rclcpp
  rcm_clients
  geometry_msgs
  tf2
  rcm_clients
  std_msgs
  fmt
)

target_link_libraries(
  voice_command_action
  own_controller
  tts_action_client
  fmt::fmt
)

ament_target_dependencies(
  own_controller
  rcm_clients
  geometry_msgs
  rcm_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  controller_manager_msgs
  moveit_msgs
  std_srvs
)

target_link_libraries(
  voice_command_client
  voice_command_action
)

target_link_libraries(voice_command_test
  voice_command_client
  voice_command_action
  own_controller
  fmt::fmt
)

target_link_libraries(voice_command_test_v2
  voice_command_client
  voice_command_action
  own_controller
  fmt::fmt
)

target_link_libraries(voice_command_test_listening
  voice_command_client
  voice_command_action
  own_controller
  fmt::fmt
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

ament_export_targets(co_manipulation_pkgTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(voice_command_client)
ament_export_libraries(tts_action_client)
ament_export_libraries(voice_command_action)

ament_export_dependencies(
  geometry_msgs
  rclcpp
  rclcpp_action
  rcm_msgs
  rcm_clients)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  voice_command_client
  EXPORT co_manipulation_pkgTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
  
install(TARGETS
  tts_action_client
  EXPORT co_manipulation_pkgTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

  install(TARGETS
  voice_command_action
  own_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

  install(TARGETS
  voice_command_test
  DESTINATION lib/${PROJECT_NAME})
  
  install(TARGETS
  voice_command_test_v2
  DESTINATION lib/${PROJECT_NAME})
  
  install(TARGETS
  voice_command_test_listening
  DESTINATION lib/${PROJECT_NAME})

  install(TARGETS
  own_controller
  voice_command_action
  EXPORT co_manipulation_pkgTargets
  )
ament_package()
