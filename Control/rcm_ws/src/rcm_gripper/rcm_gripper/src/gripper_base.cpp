#include <functional>
#include <future>
#include "rcm_gripper/gripper_base.hpp"

namespace rcm_gripper
{
  void BasicGripper::init_base_services(){
    // Init services
    base_service_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    close_service_ = node_->create_service<std_srvs::srv::Trigger>("rcm_gripper/grasp", 
      std::bind(&BasicGripper::close_hand_cb, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(),
      base_service_callback_group_);
    
    open_service_ = node_->create_service<std_srvs::srv::Trigger>("rcm_gripper/release", 
      std::bind(&BasicGripper::open_hand_cb, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(),
      base_service_callback_group_);

    RCLCPP_INFO(node_->get_logger(), "Gripper services created!");
  }

  void BasicGripper::close_hand_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    (void)request;

    auto logger = node_->get_logger();
    bool success = false;
    RCLCPP_INFO_STREAM(logger, "Request received: Close gripper");
    
    if(rclcpp::ok()){
      std::future<bool> result = std::async(std::launch::async, &BasicGripper::close, this);
      success = result.get();
    }
    
    response->success = success;

    RCLCPP_INFO(logger, "Close gripper: %s", response->success ? "SUCCESS" : "FAILED");
  }

  void BasicGripper::open_hand_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;

    auto logger = node_->get_logger();
    bool success = false;
    RCLCPP_INFO_STREAM(logger, "Request received: Open gripper");
    
    if(rclcpp::ok()){
      std::future<bool> result = std::async(std::launch::async, &BasicGripper::open, this);
      success = result.get();
    }
    
    response->success = success;
    RCLCPP_INFO(logger, "Close gripper: %s", response->success ? "SUCCESS" : "FAILED");
  }
} // namespace rcm_gripper