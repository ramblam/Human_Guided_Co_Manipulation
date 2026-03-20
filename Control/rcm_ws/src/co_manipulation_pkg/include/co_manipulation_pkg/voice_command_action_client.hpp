#ifndef VOICE_COMMAND_CLIENT_HPP
#define VOICE_COMMAND_CLIENT_HPP
#include <future>
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rcm_msgs/action/voice_command.hpp"

namespace co_manipulation_pkg {
  class VoiceCommandClient {
    public:
      using VoiceCommand = rcm_msgs::action::VoiceCommand;
      using GoalHandleVoiceCommand = rclcpp_action::ClientGoalHandle<VoiceCommand>;
      VoiceCommandClient(std::shared_ptr<rclcpp::Node>& node);
      bool send_voice_command(std::string command);
   
    protected:
      bool wait_for_result(VoiceCommand::Goal goal_msg);
      void voice_command_response_callback(const GoalHandleVoiceCommand::SharedPtr & goal_handle);
      void voice_command_feedback_callback(GoalHandleVoiceCommand::SharedPtr,
        const std::shared_ptr<const VoiceCommand::Feedback> feedback);
      void voice_command_result_callback(const GoalHandleVoiceCommand::WrappedResult & result);
      std::shared_ptr<rclcpp::Node> node_;
      std::string action_name_;
      rclcpp_action::Client<VoiceCommand>::SendGoalOptions voice_command_options_;
      rclcpp_action::Client<VoiceCommand>::SharedPtr voice_command_client_;
  };
} // namespace co_manipulation_pkg

#endif