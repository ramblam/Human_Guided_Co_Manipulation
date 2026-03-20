#ifndef CO_MANIPULATION_VOICE_ACTION_
#define CO_MANIPULATION_VOICE_ACTION_

#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "co_manipulation_pkg/own_controller.hpp"
#include "co_manipulation_pkg/tts_action_client.hpp"
#include "rcm_msgs/action/voice_command.hpp"
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/empty.hpp>

class VoiceCommandAction {
    public:
        using VoiceCommand = rcm_msgs::action::VoiceCommand;
        using GoalHandleVoiceCommand = rclcpp_action::ServerGoalHandle<VoiceCommand>;
        VoiceCommandAction(std::shared_ptr<rclcpp::Node>& node, std::shared_ptr<OwnController>& controller);
        rclcpp_action::Server<VoiceCommand>::SharedPtr voice_command_server_;

    private:
        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<OwnController> controller_;
        co_manipulation_pkg::TtsActionClient tts_action_client_;
        using SetBool = std_srvs::srv::SetBool;
        std::atomic<bool> voice_enabled_{false};
        rclcpp::Service<SetBool>::SharedPtr enable_voice_srv_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr end_session_pub_;

        rclcpp_action::GoalResponse voice_command_goal_received_cb( const rclcpp_action::GoalUUID & uuid,
                                                        std::shared_ptr<const VoiceCommand::Goal> goal);
        rclcpp_action::CancelResponse voice_command_goal_cancelled_cb(
                                        const std::shared_ptr<GoalHandleVoiceCommand> goal_handle);
        
        void voice_command_goal_accepted_cb(const std::shared_ptr<GoalHandleVoiceCommand> goal_handle);
        void voice_command_goal_feedback(const std::shared_ptr<GoalHandleVoiceCommand> goal_handle);
        void voice_command_goal_execute(const std::shared_ptr<GoalHandleVoiceCommand> goal_handle);
        void enableVoiceCb(const std::shared_ptr<SetBool::Request> req, std::shared_ptr<SetBool::Response> res);
};

#endif
