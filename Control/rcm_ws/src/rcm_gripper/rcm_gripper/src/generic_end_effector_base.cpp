#include <functional>
#include <future>
#include "rcm_gripper/generic_end_effector_base.hpp"

namespace rcm_gripper
{
  void GenericEndEffector::init_base(std::shared_ptr<rclcpp::Node> node){
    node_ = node;
  }

  void GenericEndEffector::init_base_services(){

    // Init services
    base_service_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    close_service_ = node_->create_service<std_srvs::srv::Trigger>("rcm_gripper/grasp", 
      std::bind(&GenericEndEffector::close_hand_cb, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(),
      base_service_callback_group_);
    
    open_service_ = node_->create_service<std_srvs::srv::Trigger>("rcm_gripper/release", 
      std::bind(&GenericEndEffector::open_hand_cb, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(),
      base_service_callback_group_);

    RCLCPP_INFO_STREAM(node_->get_logger(), "Gripper services created!");
  }

  void GenericEndEffector::init_base_action(){
    using namespace std::placeholders;

    end_effector_cmd_server_ = rclcpp_action::create_server<ControlEndEffector>(
      node_,
      "rcm_gripper/control_end_effector",
      std::bind(&GenericEndEffector::end_effector_goal_received_cb, this, _1, _2),
      std::bind(&GenericEndEffector::end_effector_goal_cancelled_cb, this, _1),
      std::bind(&GenericEndEffector::end_effector_goal_accepted_cb, this, _1));
  }

  void GenericEndEffector::close_hand_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    (void)request;

    using namespace std::placeholders;

    auto logger = node_->get_logger();
    bool success = false;
    RCLCPP_INFO_STREAM(logger, "Request received: Close gripper");
    
    if(rclcpp::ok()){
      std::future<bool> result = std::async(std::launch::async, &GenericEndEffector::grasp, this, 5.0);
      success = result.get();
    }
    
    response->success = success;

    RCLCPP_INFO(logger, "Close gripper: %s", response->success ? "SUCCESS" : "FAILED");
  }

  void GenericEndEffector::open_hand_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;

    using namespace std::placeholders;

    auto logger = node_->get_logger();
    bool success = false;
    RCLCPP_INFO_STREAM(logger, "Request received: Open gripper");
    
    if(rclcpp::ok()){
      std::future<bool> result = std::async(std::launch::async, &GenericEndEffector::release, this);
      success = result.get();
    }
    
    response->success = success;
    RCLCPP_INFO(logger, "Close gripper: %s", response->success ? "SUCCESS" : "FAILED");
  }

  /* Implementation of Control End Effector Action:
  * - goal_received_cb()
  * - goal_cancelled_cb()
  * - goal_accepted_cb()
  * - execute() is a virtual function, to be implemented on a plugin by plugin basis
  */

  rclcpp_action::GoalResponse GenericEndEffector::end_effector_goal_received_cb( const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ControlEndEffector::Goal> goal){
    (void)uuid; // Unused parameter
    (void)goal; // Unused parameter

    auto logger = node_->get_logger();
    RCLCPP_INFO_STREAM(logger, "Received end-effector goal");
    
    if(!goal_can_be_accepted_) {
      RCLCPP_INFO_STREAM(node_->get_logger(), "Rejected goal");
      return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO_STREAM(node_->get_logger(), "Accepted goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse GenericEndEffector::end_effector_goal_cancelled_cb(
  const std::shared_ptr<GoalHandleControlEndEffector> goal_handle){
    (void)goal_handle; // Unused parameter

    if(!goal_can_be_cancelled_) {
      RCLCPP_INFO_STREAM(node_->get_logger(), "Rejected request to cancel goal");
      return rclcpp_action::CancelResponse::REJECT;
    }

    RCLCPP_INFO_STREAM(node_->get_logger(), "Accepted request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void GenericEndEffector::end_effector_goal_accepted_cb(const std::shared_ptr<GoalHandleControlEndEffector> goal_handle){
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&GenericEndEffector::end_effector_goal_execute, this, _1), goal_handle}.detach();                                
  }

} // namespace rcm_gripper