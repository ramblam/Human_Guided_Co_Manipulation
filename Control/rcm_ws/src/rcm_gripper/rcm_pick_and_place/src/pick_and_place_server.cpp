#include <cstdio>
#include <memory>
#include <vector>
#include <math.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <rclcpp/rclcpp.hpp>
#include "rcm_pick_and_place/pap_action.hpp"
#include "rcm_pick_and_place/pap_services.hpp"
#include "rcm_pick_and_place/pap_controller.hpp"
#include "rcm_clients/cartesian_movement_client.hpp"
#include "rcm_clients/get_pose_client.hpp"

using namespace std::chrono_literals;

bool test(std::shared_ptr<PickAndPlaceController>& controller, rclcpp::Node::SharedPtr& node){
  auto logger = node->get_logger();

  RCLCPP_INFO(logger, "Test sequence started");

  rcm_clients::GetPoseClient get_pose_client(node);
  rcm_clients::CartesianMovementClient cartesian_movement_client(node);

  std::vector<geometry_msgs::msg::Pose> poses; 

  geometry_msgs::msg::Pose starting_pose;
  geometry_msgs::msg::Pose target_pose1;
  geometry_msgs::msg::Pose target_pose2;

  if(!get_pose_client.update_pose(starting_pose)){
    RCLCPP_ERROR(logger, "Failed to update pose");
    return false;
  }

  poses.push_back(starting_pose);

  tf2::Quaternion target_q1(0, 0, 0, 1);
  tf2::Vector3 rot_axis(0, 1, 0);
  target_q1.setRotation(rot_axis, 0.15 * M_PI);

  target_pose1 = starting_pose;
  target_pose1.position.y += 0.15;
  target_pose1.position.z -= 0.15;
  
  target_pose1.orientation.x = target_q1.x();
  target_pose1.orientation.y = target_q1.y();
  target_pose1.orientation.z = target_q1.z();
  target_pose1.orientation.w = target_q1.w();

  tf2::Quaternion target_q2(0, 0, 0, 1);
  tf2::Vector3 rot_axis2(1, 0, 0);
  target_q2.setRotation(rot_axis2, -0.2 * M_PI);

  target_pose2 = starting_pose;
  target_pose2.position.y -= 0.15;
  target_pose2.position.z -= 0.15;

  target_pose2.orientation.x = target_q2.x();
  target_pose2.orientation.y = target_q2.y();
  target_pose2.orientation.z = target_q2.z();
  target_pose2.orientation.w = target_q2.w();

  // Simple pick and place
  if(!controller->pick(target_pose1, 0.15, true)){
    RCLCPP_ERROR(logger, "Failed to pick");
    return false;
  }

  if(!controller->place(target_pose2, 0.15, true)){
    RCLCPP_ERROR(logger, "Failed to place");
    return false;
  };

  if(!cartesian_movement_client.execute_cartesian_trajectory(poses, 0.1)){
    RCLCPP_ERROR(logger, "Failed to return to starting position");
    return false;
  }

  // Advanced pick and place

  return true;
}

int main(int argc, char ** argv)
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);

  std::string node_name = "rcm_pick_and_place";
  rclcpp::NodeOptions node_options;

  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>(node_name, node_options);

  RCLCPP_INFO(node->get_logger(), "RCM Pick and Place interface starting");

  // Executor to spin the node
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread(std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, &executor));
  
  // Create controller
  std::shared_ptr<PickAndPlaceController> controller = std::make_shared<PickAndPlaceController>(node);

  // Start PAP actions and services
  PickAndPlaceAction pap_actions(node, controller);
  PickAndPlaceServices pap_services(node, controller);

  // Optional test, configurable by parameter
  bool tests_enabled = false;
  node->declare_parameter("tests_enabled", false);
  node->get_parameter("tests_enabled", tests_enabled);

  if(tests_enabled){
    if(!test(controller, node)){
      RCLCPP_INFO(node->get_logger(), "Test function failed, exiting");
      rclcpp::shutdown();
    }
    else{
      RCLCPP_INFO(node->get_logger(), "Test function successful, RCM MoveIt interface ready");
    }
  }
  else {
    RCLCPP_INFO(node->get_logger(), "RCM Pick and Place interface ready");
  }

  // Shutdown ROS
  executor_thread.join();
  rclcpp::shutdown();

  return 0;
}
