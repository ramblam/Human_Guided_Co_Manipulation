#include "rcm_clients/pose_movement_client.hpp"

namespace rcm_clients {
  PoseMovementClient::PoseMovementClient(std::shared_ptr<rclcpp::Node>& node){
    using namespace std::placeholders;

    action_name_ = "rcm_moveit/actions/pose_movement";
    node_ = node;
    pose_movement_client_ = rclcpp_action::create_client<PoseMovementCommand>(
        node_,
        action_name_);
      
    pose_movement_options_ = rclcpp_action::Client<PoseMovementCommand>::SendGoalOptions();
    pose_movement_options_.goal_response_callback =
      std::bind(&PoseMovementClient::pose_movement_response_callback, this, _1);
    pose_movement_options_.feedback_callback =
      std::bind(&PoseMovementClient::pose_movement_feedback_callback, this, _1, _2);
    pose_movement_options_.result_callback =
      std::bind(&PoseMovementClient::pose_movement_result_callback, this, _1);
  }
    
  bool PoseMovementClient::execute_pose_movement(geometry_msgs::msg::Pose pose,
    double velocity, double timeout_ms, std::string mode){
    using namespace std::chrono_literals;
    
    auto logger = node_->get_logger();
    bool success = false;

    // Wait for action server to be ready
    for(int i=0; i<10; i++){
      RCLCPP_INFO(logger, "Waiting for %s to be available...", action_name_.c_str());

      if(!pose_movement_client_->wait_for_action_server(1s)) {
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

    auto pose_movement_msg = PoseMovementCommand::Goal();
    pose_movement_msg.pose = pose;
    pose_movement_msg.mode = mode;
    pose_movement_msg.velocity = velocity;
    pose_movement_msg.timeout_ms = timeout_ms;

    auto goal_handle_future = pose_movement_client_->async_send_goal(pose_movement_msg, pose_movement_options_);
    
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

    auto result_future = pose_movement_client_->async_get_result(goal_handle_future.get());
  
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

  void PoseMovementClient::pose_movement_response_callback(const GoalHandlePoseMovementCommand::SharedPtr & goal_handle){
    auto logger = node_->get_logger();
    if (!goal_handle) {
      RCLCPP_ERROR_STREAM(logger, "Goal was rejected by " << action_name_);
    } else {
      RCLCPP_INFO_STREAM(logger, "Goal was accepted by " << action_name_);
    }
  }

  void PoseMovementClient::pose_movement_feedback_callback(GoalHandlePoseMovementCommand::SharedPtr,
    const std::shared_ptr<const PoseMovementCommand::Feedback> feedback){
    RCLCPP_INFO(node_->get_logger(), "Feedback: Time: %f ms, State: %s", feedback->ms_waited_until_now, feedback->state.c_str());
  }

  void PoseMovementClient::pose_movement_result_callback(const GoalHandlePoseMovementCommand::WrappedResult & result){
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
