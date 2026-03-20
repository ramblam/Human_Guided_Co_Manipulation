cmake_minimum_required(VERSION 3.8)
project(rcm_cartesian_impedance_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(Boost 1.49 REQUIRED)
find_package(PkgConfig)

# Find RBDyn library and dependencies
# pkg_search_module(Eigen3 REQUIRED eigen3)
# pkg_check_modules(mc_rbdyn_urdf REQUIRED mc_rbdyn_urdf)
# pkg_check_modules(RBDyn REQUIRED RBDyn)
# pkg_check_modules(SpaceVecAlg REQUIRED SpaceVecAlg)
# pkg_check_modules(tinyxml2 REQUIRED tinyxml2)


find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)
#find_package(dynamic_reconfigure REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
#find_package(message_generation REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(joint_trajectory_controller REQUIRED)

generate_parameter_library(
  cartesian_impedance_controller_ros_parameters # cmake target name for the parameter library
  src/cartesian_impedance_controller_ros_parameters.yaml # path to input yaml file
)

pluginlib_export_plugin_description_file(controller_interface plugins.xml)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControllerConfig.msg"
  "msg/ControllerState.msg"
  DEPENDENCIES
  builtin_interfaces
  std_msgs
  geometry_msgs
  sensor_msgs
)

###########
## Build ##
###########

# Sets build type to "Release" in case no build type has not been set before. This is necessary to run this controller at 1 kHz.
# On our RT system with an i5 11th Gen processor this reduces update evaluations from about 800 to 300 microseconds
if(NOT CMAKE_BUILD_TYPE) 
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

# Core library

add_library(cartesian_impedance_controller src/cartesian_impedance_controller.cpp)

ament_target_dependencies(
  cartesian_impedance_controller
  pluginlib
  Eigen3
)

target_compile_options(cartesian_impedance_controller PUBLIC -std=c++17)

target_include_directories(cartesian_impedance_controller PUBLIC 
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${Eigen3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${SpaceVecAlg_INCLUDE_DIRS} 
  ${RBDyn_INCLUDE_DIRS} 
  ${mc_rbdyn_urdf_INCLUDE_DIRS})

target_link_libraries(cartesian_impedance_controller ${Eigen3_LIBRARIES})

# ROS Integration
add_library(cartesian_impedance_controller_ros src/cartesian_impedance_controller_ros.cpp)

# Msgs
rosidl_target_interfaces(cartesian_impedance_controller_ros
      ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_target_dependencies(
  cartesian_impedance_controller_ros
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  realtime_tools
  controller_interface
  pluginlib
  geometry_msgs
  trajectory_msgs
  rclcpp
  Eigen3
  control_msgs
  joint_trajectory_controller
)

target_compile_options(cartesian_impedance_controller_ros PUBLIC -std=c++17)

target_include_directories(cartesian_impedance_controller_ros PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>" 
  ${catkin_INCLUDE_DIRS} 
  ${Eigen3_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIRS} 
  ${tinyxml2_INCLUDE_DIRS} 
  ${SpaceVecAlg_INCLUDE_DIRS} 
  ${RBDyn_INCLUDE_DIRS} 
  ${mc_rbdyn_urdf_INCLUDE_DIRS})

target_link_libraries(cartesian_impedance_controller_ros 
  cartesian_impedance_controller 
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES} 
  ${tinyxml2_LIBRARIES} 
  RBDyn 
  mc_rbdyn_urdf 
  ${Eigen3_LIBRARIES}
  cartesian_impedance_controller_ros_parameters)

# Install libraries
install(TARGETS
  cartesian_impedance_controller
  cartesian_impedance_controller_ros
  cartesian_impedance_controller_ros_parameters
  EXPORT export_rcm_controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

# Pluginlib
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)

# Generate parameter library
ament_export_targets(export_rcm_controllers HAS_LIBRARY_TARGET)

ament_package()
