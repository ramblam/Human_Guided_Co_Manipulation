#include "co_manipulation_pkg/voice_command_action_client.hpp"

namespace co_manipulation_pkg {
  VoiceCommandClient::VoiceCommandClient(std::shared_ptr<rclcpp::Node>& node){
    using namespace std::placeholders;

    action_name_ = "/rcm/voice_command";
    node_ = node;
    voice_command_client_ = rclcpp_action::create_client<VoiceCommand>(
        node_,
        action_name_);
      
    voice_command_options_ = rclcpp_action::Client<VoiceCommand>::SendGoalOptions();
    voice_command_options_.goal_response_callback =
      std::bind(&VoiceCommandClient::voice_command_response_callback, this, _1);
    voice_command_options_.feedback_callback =
      std::bind(&VoiceCommandClient::voice_command_feedback_callback, this, _1, _2);
    voice_command_options_.result_callback =
      std::bind(&VoiceCommandClient::voice_command_result_callback, this, _1);
  }
    
 bool VoiceCommandClient::send_voice_command(std::string command){
    using namespace std::chrono_literals;
    
    auto logger = node_->get_logger();
    bool success = false;
    auto voice_command_msg = VoiceCommand::Goal();
    voice_command_msg.voice_command = command;

    success = wait_for_result(voice_command_msg);

    return success;
  }

  bool VoiceCommandClient::wait_for_result(VoiceCommand::Goal goal_msg){
    using namespace std::chrono_literals;
    
    auto logger = node_->get_logger();
    bool success = false;

    auto goal_handle_future = voice_command_client_->async_send_goal(goal_msg, voice_command_options_);
    
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

    auto result_future = voice_command_client_->async_get_result(goal_handle_future.get());
  
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

  void VoiceCommandClient::voice_command_response_callback(const GoalHandleVoiceCommand::SharedPtr & goal_handle){
    auto logger = node_->get_logger();
    if (!goal_handle) {
      RCLCPP_ERROR_STREAM(logger, "Goal was rejected by " << action_name_);
    } else {
      RCLCPP_INFO_STREAM(logger, "Goal was accepted by " << action_name_);
    }
  }

  void VoiceCommandClient::voice_command_feedback_callback(GoalHandleVoiceCommand::SharedPtr,
    const std::shared_ptr<const VoiceCommand::Feedback> feedback){
    RCLCPP_INFO(node_->get_logger(), "State: %s", feedback->state.c_str());
  }

  void VoiceCommandClient::voice_command_result_callback(const GoalHandleVoiceCommand::WrappedResult & result){
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
} // namespace co_manipulation_pkg
