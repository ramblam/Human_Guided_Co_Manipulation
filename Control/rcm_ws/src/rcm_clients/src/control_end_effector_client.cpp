#include "rcm_clients/control_end_effector_client.hpp"

namespace rcm_clients {
  ControlEndEffectorClient::ControlEndEffectorClient(std::shared_ptr<rclcpp::Node>& node){
    using namespace std::placeholders;

    action_name_ = "/rcm_gripper/control_end_effector";
    node_ = node;
    control_end_effector_client_ = rclcpp_action::create_client<ControlEndEffector>(
        node_,
        action_name_);
      
    control_end_effector_options_ = rclcpp_action::Client<ControlEndEffector>::SendGoalOptions();
    control_end_effector_options_.goal_response_callback =
      std::bind(&ControlEndEffectorClient::control_end_effector_response_callback, this, _1);
    control_end_effector_options_.feedback_callback =
      std::bind(&ControlEndEffectorClient::control_end_effector_feedback_callback, this, _1, _2);
    control_end_effector_options_.result_callback =
      std::bind(&ControlEndEffectorClient::control_end_effector_result_callback, this, _1);
  }
    
  bool ControlEndEffectorClient::control_end_effector(std::vector<std::string> io_name, std::vector<long int> io_state,
        double width, double force, double angle){
    using namespace std::chrono_literals;
    
    auto logger = node_->get_logger();
    bool success = false;

    // Wait for action server to be ready
    for(int i=0; i<10; i++){
      RCLCPP_INFO(logger, "Waiting for %s to be available...", action_name_.c_str());

      if(!control_end_effector_client_->wait_for_action_server(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(logger, "Interrupted while waiting for %s. Exiting.", action_name_.c_str());
            return success;
        }
        else if (i>8){
            RCLCPP_ERROR(logger, "Timed out while waiting for %s. Exiting.", action_name_.c_str());
            return success;
        }
      }
      else {
        break;
      }
    }

    auto control_end_effector_msg = ControlEndEffector::Goal();
    control_end_effector_msg.io_name = io_name;
    control_end_effector_msg.io_state = io_state;
    control_end_effector_msg.width = width;
    control_end_effector_msg.force = force;
    control_end_effector_msg.angle = angle;

    auto goal_handle_future = control_end_effector_client_->async_send_goal(control_end_effector_msg, control_end_effector_options_);
    
    // Wait for goal to be accepted
    for(int i=0; i<10; i++){
      RCLCPP_INFO(logger, "Waiting for response from %s...", action_name_.c_str());

      if(goal_handle_future.wait_for(1s) == std::future_status::timeout) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(logger, "Interrupted while waiting for %s. Exiting.", action_name_.c_str());
            return success;
        }
        else if (i>8){
            RCLCPP_ERROR(logger, "Timed out while waiting for %s. Exiting.", action_name_.c_str());
            return success;
        }
      }
      else {
        break;
      }
    }

    auto result_future = control_end_effector_client_->async_get_result(goal_handle_future.get());
  
    // Wait for result
    result_future.wait();
    for(int i=0; i<10; i++){
      if(result_future.wait_for(1s) == std::future_status::timeout) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(logger, "Interrupted while waiting to get a result from %s. Exiting.", action_name_.c_str());
            return success;
        }
        else if (i>8){
            RCLCPP_ERROR(logger, "Timed out while waiting to get a result from %s. Exiting.", action_name_.c_str());
            return success;
        }
      }
      else {
        break;
      }
    }

    auto result = result_future.get();

    if(result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      success = result.result->success;
    }

    return success;
  }

  void ControlEndEffectorClient::control_end_effector_response_callback(const GoalHandleControlEndEffector::SharedPtr & goal_handle){
    auto logger = node_->get_logger();
    if (!goal_handle) {
      RCLCPP_ERROR_STREAM(logger, "Goal was rejected by " << action_name_);
    } else {
      RCLCPP_INFO_STREAM(logger, "Goal was accepted by " << action_name_);
    }
  }

  void ControlEndEffectorClient::control_end_effector_feedback_callback(GoalHandleControlEndEffector::SharedPtr,
    const std::shared_ptr<const ControlEndEffector::Feedback> feedback){
    RCLCPP_INFO(node_->get_logger(), "State: %s", feedback->state.c_str());
  }

  void ControlEndEffectorClient::control_end_effector_result_callback(const GoalHandleControlEndEffector::WrappedResult & result){
    auto logger = node_->get_logger();

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(logger, "Goal completed: %s", result.result->success ? "SUCCESS" : "FAILED");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(logger, "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(logger, "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(logger, "Unknown result code");
        break;
    }
  }
} // namespace rcm_clients
