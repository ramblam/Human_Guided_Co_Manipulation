#include <functional>
#include <rclcpp/logging.hpp>
#include "rcm_pick_and_place/pap_services.hpp"

PickAndPlaceServices::PickAndPlaceServices(std::shared_ptr<rclcpp::Node>& node, 
  std::shared_ptr<PickAndPlaceController>& controller){
  node_ = node;
  controller_ = controller;

  // Creating services on the received node
  pick_service_ = node_->create_service<rcm_msgs::srv::Target>("rcm_pick_and_place/pick", 
              std::bind(&PickAndPlaceServices::pick_cb, this, std::placeholders::_1, std::placeholders::_2));
  
  place_service_ = node_->create_service<rcm_msgs::srv::Target>("rcm_pick_and_place/place", 
              std::bind(&PickAndPlaceServices::place_cb, this, std::placeholders::_1, std::placeholders::_2));               

  RCLCPP_INFO(node_->get_logger(), "PAP Services created!");
}

void PickAndPlaceServices::pick_cb(const std::shared_ptr<rcm_msgs::srv::Target::Request> request,
                                std::shared_ptr<rcm_msgs::srv::Target::Response> response){
  RCLCPP_INFO(node_->get_logger(), "Request received: Pick");
  bool success = false;
  geometry_msgs::msg::Pose target_pose = request->target_pose;
  
  success = controller_->pick(target_pose, 0.2, false);

  response->success = success;    

  RCLCPP_INFO(node_->get_logger(), "Request served: Pick");
}

void PickAndPlaceServices::place_cb(const std::shared_ptr<rcm_msgs::srv::Target::Request> request,
                              std::shared_ptr<rcm_msgs::srv::Target::Response> response){
  RCLCPP_INFO(node_->get_logger(), "Request received: Place");
  bool success = false;
  geometry_msgs::msg::Pose target_pose = request->target_pose;

  success = controller_->place(target_pose, 0.2, false);

  response->success = success; 

  RCLCPP_INFO(node_->get_logger(), "Request served: Place");
}