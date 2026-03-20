#include <thread>
#include <atomic>
#include <sstream>
#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <fmt/core.h>
#include "co_manipulation_pkg/voice_command_action.hpp"

VoiceCommandAction::VoiceCommandAction(std::shared_ptr<rclcpp::Node>& node, std::shared_ptr<OwnController>& controller)
: node_(node), controller_(controller), tts_action_client_(node)
{
    using namespace std::placeholders;

    voice_command_server_ = rclcpp_action::create_server<VoiceCommand>(
      node_,
      "/rcm/voice_command",
      std::bind(&VoiceCommandAction::voice_command_goal_received_cb, this, _1, _2),
      std::bind(&VoiceCommandAction::voice_command_goal_cancelled_cb, this, _1),
      std::bind(&VoiceCommandAction::voice_command_goal_accepted_cb, this, _1));

    enable_voice_srv_ = node_->create_service<SetBool>("/enable_voice_commands",std::bind(&VoiceCommandAction::enableVoiceCb,
    this,
    std::placeholders::_1,
    std::placeholders::_2));

    end_session_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/rcm/voice_session/end",10);

}

rclcpp_action::GoalResponse VoiceCommandAction::voice_command_goal_received_cb( const rclcpp_action::GoalUUID & uuid,
                                                        std::shared_ptr<const VoiceCommand::Goal> goal){
    (void)uuid; // Unused parameter
    (void)goal; // Unused parameter

    auto logger = node_->get_logger();

    if (!voice_enabled_) {
        RCLCPP_WARN(node_->get_logger(), "Voice command rejected (voice control disabled)");
        return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(node_->get_logger(), "Voice command accepted");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse VoiceCommandAction::voice_command_goal_cancelled_cb(
                                const std::shared_ptr<GoalHandleVoiceCommand> goal_handle){
    (void)goal_handle; // Unused parameter
    RCLCPP_INFO(node_->get_logger(), "Received request to cancel voice command");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void VoiceCommandAction::voice_command_goal_accepted_cb(const std::shared_ptr<GoalHandleVoiceCommand> goal_handle){
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&VoiceCommandAction::voice_command_goal_execute, this, _1), goal_handle}.detach();                                
}

void VoiceCommandAction::enableVoiceCb(const std::shared_ptr<SetBool::Request> req, std::shared_ptr<SetBool::Response> res)
{
  voice_enabled_.store(req->data, std::memory_order_relaxed);

  res->success = true;
  res->message = req->data
    ? "Voice commands enabled"
    : "Voice commands disabled";

  RCLCPP_INFO(node_->get_logger(), "%s", res->message.c_str());
}

void VoiceCommandAction::voice_command_goal_execute(const std::shared_ptr<GoalHandleVoiceCommand> goal_handle){
    using namespace std::chrono_literals;

    auto logger = node_->get_logger();
    //This handles the case where voice is disabled mid-execution (Should be rare, though)
    if (!voice_enabled_) {
        auto result = std::make_shared<VoiceCommand::Result>();
        result->success = false;
        goal_handle->abort(result);
        return;
    }
    
    // Action setup
    RCLCPP_INFO(logger, "Processing voice command");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<VoiceCommand::Feedback>();
    auto result = std::make_shared<VoiceCommand::Result>();
    result->success = false;
    geometry_msgs::msg::Pose target_pose;
    std::vector<geometry_msgs::msg::Pose> poses;

    auto to_upper = [](std::string &s){
        std::transform(s.begin(), s.end(), s.begin(),
                        [](unsigned char c){ return std::toupper(c); });

    };

    auto msg = goal->voice_command;
    RCLCPP_INFO(logger, "Msg here: %s", msg.c_str());

    std::istringstream iss(msg);
    std::vector<std::string> tokens;
    std::string token;
    double move_dist = 0.1;
    int dist_msg = 10;
    std::string unit = "centimeters";

    while (iss >> token) {
        to_upper(token);
        tokens.push_back(token);
    }

    if (tokens[0] == "MOVE") {
        if (tokens.size() == 4) {
            if (tokens[3] == "MILLIMETERS" || tokens[3] == "MILLIMETRES") {
                move_dist = std::stod(tokens[2])/1000;
                dist_msg = std::stoi(tokens[2]);
                unit = "millimeters";
            } else if (tokens[3] == "CENTIMETERS" || tokens[3] == "CENTIMETRES") {
                move_dist = std::stod(tokens[2])/100;
                dist_msg = std::stoi(tokens[2]);
                unit = "centimeters";
            } else {
                tts_action_client_.send_tts_msg("Please specify unit");
                unit = "";
            }
        } else if (tokens.size() == 3) {
            move_dist = std::stod(tokens[2])/1000;
            dist_msg = std::stoi(tokens[2]);
            unit = "millimeters";
        } else if (tokens.size() == 2) {
            move_dist = 0.10;
            unit = "millimeters";
            dist_msg = static_cast<int>(std::round(1000 * move_dist));
        }

        if (tokens[1] == "DOWN") {
            if(!controller_->update_current_pose(target_pose)){
                RCLCPP_ERROR(logger, "Failed to update pose");
                feedback->state = "Failure while updating pose";
                goal_handle->publish_feedback(feedback);
                return;
            }
            RCLCPP_INFO(logger, "Moving down");
            feedback->state = "Moving down";
            goal_handle->publish_feedback(feedback);
            tts_action_client_.send_tts_msg(fmt::format("Moving down {} {}.",dist_msg, unit));
            target_pose.position.z -= move_dist;
            poses.push_back(target_pose);

            if(!controller_->move_to_pose(target_pose, 0.2, 15000.)) {
                RCLCPP_ERROR(logger, "Failure while moving down");
                feedback->state = "Failure while moving down";
                goal_handle->publish_feedback(feedback);
                tts_action_client_.send_tts_msg("Failure while moving down");
                goal_handle->succeed(result);
                return;
            }
            poses.clear();
        } else if (tokens[1] == "UP") {
            if(!controller_->update_current_pose(target_pose)){
                RCLCPP_ERROR(logger, "Failed to update pose");
                feedback->state = "Failure while updating pose";
                goal_handle->publish_feedback(feedback);
                return;
            }
            RCLCPP_INFO(logger, "Moving up");
            feedback->state = "Moving up";
            goal_handle->publish_feedback(feedback);
            tts_action_client_.send_tts_msg(fmt::format("Moving up {} {}.",dist_msg, unit));
            target_pose.position.z += move_dist;
            poses.push_back(target_pose);

            if(!controller_->move_to_pose(target_pose, 0.2, 15000.)) {
                RCLCPP_ERROR(logger, "Failure while moving up");
                feedback->state = "Failure while moving up";
                goal_handle->publish_feedback(feedback);
                tts_action_client_.send_tts_msg("Failure while moving up");
                goal_handle->succeed(result);
                return;
            }
            poses.clear();
        } else if (tokens[1] == "RIGHT") {
            if(!controller_->update_current_pose(target_pose)){
                RCLCPP_ERROR(logger, "Failed to update pose");
                feedback->state = "Failure while updating pose";
                goal_handle->publish_feedback(feedback);
                return;
            }
            RCLCPP_INFO(logger, "Moving right");
            feedback->state = "Moving right";
            goal_handle->publish_feedback(feedback);
            tts_action_client_.send_tts_msg(fmt::format("Moving right {} {}.",dist_msg, unit));
            target_pose.position.y += move_dist;
            poses.push_back(target_pose);

            if(!controller_->move_to_pose(target_pose, 0.2, 15000.)) {
                RCLCPP_ERROR(logger, "Failure while moving right");
                feedback->state = "Failure while moving right";
                goal_handle->publish_feedback(feedback);
                tts_action_client_.send_tts_msg("Failure while moving right");
                goal_handle->succeed(result);
                return;
            }
            poses.clear();
        } else if (tokens[1] == "LEFT") {
            if(!controller_->update_current_pose(target_pose)){
                RCLCPP_ERROR(logger, "Failed to update pose");
                feedback->state = "Failure while updating pose";
                goal_handle->publish_feedback(feedback);
                return;
            }
            RCLCPP_INFO(logger, "Moving left");
            feedback->state = "Moving left";
            goal_handle->publish_feedback(feedback);
            tts_action_client_.send_tts_msg(fmt::format("Moving left {} {}.",dist_msg, unit));
            target_pose.position.y -= move_dist;
            poses.push_back(target_pose);

            if(!controller_->move_to_pose(target_pose, 0.2, 15000.)) {
                RCLCPP_ERROR(logger, "Failure while moving left");
                feedback->state = "Failure while moving left";
                goal_handle->publish_feedback(feedback);
                tts_action_client_.send_tts_msg("Failure while moving left");
                goal_handle->succeed(result);
                return;
            }
            poses.clear();
        } else if (tokens[1] == "FORWARD") {
            if(!controller_->update_current_pose(target_pose)){
                RCLCPP_ERROR(logger, "Failed to update pose");
                feedback->state = "Failure while updating pose";
                goal_handle->publish_feedback(feedback);
                return;
            }
            RCLCPP_INFO(logger, "Moving forward");
            feedback->state = "Moving forward";
            goal_handle->publish_feedback(feedback);
            tts_action_client_.send_tts_msg(fmt::format("Moving forward {} {}.",dist_msg, unit));
            target_pose.position.x += move_dist;
            poses.push_back(target_pose);

            if(!controller_->move_to_pose(target_pose, 0.2, 15000.)) {
                RCLCPP_ERROR(logger, "Failure while moving forward");
                feedback->state = "Failure while moving forward";
                goal_handle->publish_feedback(feedback);
                tts_action_client_.send_tts_msg("Failure while moving forward");
                goal_handle->succeed(result);
                return;
            }
            poses.clear();
        } else if (tokens[1] == "BACKWARD") {
            if(!controller_->update_current_pose(target_pose)){
                RCLCPP_ERROR(logger, "Failed to update pose");
                feedback->state = "Failure while updating pose";
                goal_handle->publish_feedback(feedback);
                return;
            }
            RCLCPP_INFO(logger, "Moving back");
            feedback->state = "Moving back";
            goal_handle->publish_feedback(feedback);
            tts_action_client_.send_tts_msg(fmt::format("Moving back {} {}.",dist_msg, unit));
            target_pose.position.x -= move_dist;
            poses.push_back(target_pose);

            if(!controller_->move_to_pose(target_pose, 0.2, 15000.)) {
                RCLCPP_ERROR(logger, "Failure while moving back");
                feedback->state = "Failure while moving back";
                goal_handle->publish_feedback(feedback);
                tts_action_client_.send_tts_msg("Failure while moving back");
                goal_handle->succeed(result);
                return;
            }
            poses.clear();
        } else if (tokens[1] == "HOME") {
            RCLCPP_INFO(logger, "Moving home");
            feedback->state = "Moving home";
            goal_handle->publish_feedback(feedback);
            tts_action_client_.send_tts_msg("Moving home");

            if(!controller_->move_home()) {
                RCLCPP_ERROR(logger, "Failure while moving home");
                feedback->state = "Failure while moving home";
                goal_handle->publish_feedback(feedback);
                tts_action_client_.send_tts_msg("Failure while moving home");
                goal_handle->succeed(result);
                return;
            }
        }
    } else if (tokens[0] == "RESIST") {
        std::string direction = tokens[2];
        std::transform(direction.begin(), direction.end(), direction.begin(),
                   [](unsigned char c){ return std::tolower(c); });


        if (tokens[1] == "MORE") {
            if (!controller_->resist_more(direction)) {
                RCLCPP_ERROR(logger, "Failure while resisting more");
                feedback->state = "Failure while resisting more";
                goal_handle->publish_feedback(feedback);
                tts_action_client_.send_tts_msg("Failure while resisting more");

                goal_handle->succeed(result);
                return;
            } else {
                RCLCPP_INFO(logger, "Resisting more");
                feedback->state = "Resisting more";
                goal_handle->publish_feedback(feedback);
                tts_action_client_.send_tts_msg("Resisting more");
            }
            
        } else if (tokens[1] == "LESS") {
            if (!controller_->resist_less(direction)) {
                RCLCPP_ERROR(logger, "Failure while resisting less");
                feedback->state = "Failure while resisting less";
                goal_handle->publish_feedback(feedback);
                tts_action_client_.send_tts_msg("Failure while resisting less");

                goal_handle->succeed(result);
                return;
            } else {
                RCLCPP_INFO(logger, "Resisting less");
                feedback->state = "Resisting less";
                goal_handle->publish_feedback(feedback);
                tts_action_client_.send_tts_msg("Resisting less");
            }
        }
    }
    if (msg == "TOOL OPEN"){
        RCLCPP_INFO(logger, "Opening the gripper");
        feedback->state = "Opening gripper";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Opening the gripper");
        if(!controller_->open_gripper()) {
            RCLCPP_ERROR(logger, "Failure while opening the gripper");
            feedback->state = "Failure while opening the gripper";
            goal_handle->publish_feedback(feedback);
            tts_action_client_.send_tts_msg("Failure while opening the gripper");
            goal_handle->succeed(result);
            return;
        }
    } else if (msg == "TOOL CLOSE") {
        RCLCPP_INFO(logger, "Closing the gripper");
        feedback->state = "Closing gripper";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Closing the gripper");
        if(!controller_->close_gripper()) {
            RCLCPP_ERROR(logger, "Failure while closing the gripper");
            feedback->state = "Failure while closing the gripper";
            goal_handle->publish_feedback(feedback);
            tts_action_client_.send_tts_msg("Failure while closing the gripper");
            goal_handle->succeed(result);
            return;
        }
         
    } else if (msg == "ACTIVATE WRIST FOLLOWING REF") {
        if(!controller_->activate_wrist_following_ref()){
            RCLCPP_ERROR(logger, "Failed to activate wrist following");
            feedback->state = "Failure while activating wrist following";
            goal_handle->publish_feedback(feedback);
            tts_action_client_.send_tts_msg("Failed to activate wrist following");
            return;
    }
        RCLCPP_INFO(logger, "Wrist following activated");
        feedback->state = "Wrist following activated";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Wrist following activated");

    } else if (msg == "ACTIVATE COMPLIANCE CONTROLLER") {
        if(!controller_->switch_to_compliance_controller()){
        RCLCPP_ERROR(logger, "Failed to activate compliance controller");
        feedback->state = "Failure while activating compliance controller";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Failed to activate compliance controller");
        return;
    }
        RCLCPP_INFO(logger, "Compliance controller activated");
        feedback->state = "Compliance controller activated";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Compliance controller activated");

    } else if (msg == "ACTIVATE POSITION CONTROLLER") {
        if(!controller_->switch_to_default_controller()){
        RCLCPP_ERROR(logger, "Failed to activate default controller");
        feedback->state = "Failure while activating default controller";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Failed to activate default controller");
        return;
    }
        RCLCPP_INFO(logger, "Default controller activated");
        feedback->state = "Default controller activated";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Default controller activated");

    } else if (msg == "DEACTIVATE WRIST FOLLOWING REF") {
        if(!controller_->deactivate_wrist_following_ref()){
        RCLCPP_ERROR(logger, "Failed to deactivate wrist following");
        feedback->state = "Failure while deactivating wrist following";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Failed to deactivate wrist following");
        return;
    }
        RCLCPP_INFO(logger, "Wrist following deactivated");
        feedback->state = "Wrist following deactivated";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Wrist following deactivated");

    } else if (msg == "ACTIVATE WRIST FOLLOWING SERVO") {
        if(!controller_->activate_wrist_following_servo()){
        RCLCPP_ERROR(logger, "Failed to activate wrist following servo");
        feedback->state = "Failure while activating wrist following servo";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Failed to activate wrist following");
        return;
    }
        RCLCPP_INFO(logger, "Wrist following servo activated");
        feedback->state = "Wrist following servo activated";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Wrist following activated");

    } else if (msg == "DEACTIVATE WRIST FOLLOWING SERVO") {
        if(!controller_->deactivate_wrist_following_servo()){
        RCLCPP_ERROR(logger, "Failed to deactivate wrist following");
        feedback->state = "Failure while deactivating wrist following servo";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Failed to deactivate wrist following");
        return;
    }
        RCLCPP_INFO(logger, "Wrist following servo deactivated");
        feedback->state = "Wrist following servo  deactivated";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Wrist following deactivated");

    } else if (msg == "ALLOW COMPLIANCE XYZ") {
        if(!controller_->allow_compliance_3d()){
        RCLCPP_ERROR(logger, "Failed to allow compliance in 3d");
        feedback->state = "Failure while allowing compliance in 3d";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Failed to allow hand guiding in 3D");
        return;
    }
        RCLCPP_INFO(logger, "Compliance allowed in 3d");
        feedback->state = "Compliance allowed in 3D";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Hand guiding allowed in 3D");

    } else if (msg == "ALLOW COMPLIANCE ALL") {
        if(!controller_->allow_compliance_6d()){
        RCLCPP_ERROR(logger, "Failed to allow compliance in 6d");
        feedback->state = "Failure while allowing compliance in 6d";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Failed to allow hand guiding in 6D");
        return;
    }
        RCLCPP_INFO(logger, "Compliance allowed in 6d");
        feedback->state = "Compliance allowed in 6D";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Hand guiding allowed in 6D");

    } else if (msg == "ALLOW COMPLIANCE PLANE") {
        if(!controller_->allow_compliance_plane()){
        RCLCPP_ERROR(logger, "Failed to allow compliance on plane");
        feedback->state = "Failure while allowing compliance on plane";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Failed to allow hand guiding on plane");
        return;
    }
        std::this_thread::sleep_for(5s);
        RCLCPP_INFO(logger, "Compliance allowed on plane");
        feedback->state = "Compliance allowed on plane";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Hand guiding allowed on plane");
} else if (msg == "ALLOW COMPLIANCE ALSO X") {
        if(!controller_->allow_compliance_also("x")){
        RCLCPP_ERROR(logger, "Failed to allow compliance also in x direction");
        feedback->state = "Failure while allowing compliance also in x direction";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Failed to allow hand guiding also in x direction");
        return;
    }
        RCLCPP_INFO(logger, "Compliance allowed also in x direction");
        feedback->state = "Compliance allowed also in x direction";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Hand guiding allowed also in x direction");
} else if (msg == "ALLOW COMPLIANCE ALSO Y") {
        if(!controller_->allow_compliance_also("y")){
        RCLCPP_ERROR(logger, "Failed to allow compliance also in y direction");
        feedback->state = "Failure while allowing compliance also in y direction";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Failed to allow hand guiding also in y direction");
        return;
    }
        RCLCPP_INFO(logger, "Compliance allowed also in y direction");
        feedback->state = "Compliance allowed also in y direction";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Hand guiding allowed also in y direction");
} else if (msg == "ALLOW COMPLIANCE ALSO Z") {
        if(!controller_->allow_compliance_also("z")){
        RCLCPP_ERROR(logger, "Failed to allow compliance also in z direction");
        feedback->state = "Failure while allowing compliance also in z direction";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Failure while allowing hand guiding also in z direction");
        return;
    }
        RCLCPP_INFO(logger, "Compliance allowed also in z direction");
        feedback->state = "Compliance allowed also in z direction";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Hand guiding allowed also in z direction");
} else if (msg == "ALLOW COMPLIANCE ONLY X") {
        if(!controller_->allow_compliance_only("x")){
        RCLCPP_ERROR(logger, "Failed to allow compliance only in x direction");
        feedback->state = "Failure while allowing compliance only in x direction";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Failed to allow hand guiding only in x direction");
        return;
    }
        RCLCPP_INFO(logger, "Compliance allowed only in x direction");
        feedback->state = "Compliance allowed only in x direction";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Hand guiding allowed only in x direction");
} else if (msg == "ALLOW COMPLIANCE ONLY Y") {
        if(!controller_->allow_compliance_only("y")){
        RCLCPP_ERROR(logger, "Failed to allow compliance only in y direction");
        feedback->state = "Failure while allowing compliance only in y direction";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Failed to allow hand guiding only in y direction");
        return;
    }
        RCLCPP_INFO(logger, "Compliance allowed only in y direction");
        feedback->state = "Compliance allowed only in y direction";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Hand guiding allowed only in y direction");
} else if (msg == "ALLOW COMPLIANCE ONLY Z") {
        if(!controller_->allow_compliance_only("z")){
        RCLCPP_ERROR(logger, "Failed to allow compliance only in z direction");
        feedback->state = "Failure while allowing compliance only in z direction";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Failed to allow hand guiding only in z direction");
        return;
    }
        RCLCPP_INFO(logger, "Compliance allowed only in z direction");
        feedback->state = "Compliance allowed only in z direction";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Hand guiding allowed only in z direction");
} else if (msg == "DISALLOW COMPLIANCE ALSO X") {
        if(!controller_->disallow_compliance_also("x")){
        RCLCPP_ERROR(logger, "Failed to disallow compliance also in x direction");
        feedback->state = "Failure while disallowing compliance also in x direction";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Failed to disallow hand guiding also in x direction");
        return;
    }
        RCLCPP_INFO(logger, "Compliance disallowed also in x direction");
        feedback->state = "Compliance disallowed also in x direction";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Hand guiding disallowed also in x direction");
} else if (msg == "DISALLOW COMPLIANCE ALSO Y") {
        if(!controller_->disallow_compliance_also("y")){
        RCLCPP_ERROR(logger, "Failed to disallow compliance also in y direction");
        feedback->state = "Failure while disallowing compliance also in y direction";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Failed to disallow hand guiding also in y direction");
        return;
    }
        RCLCPP_INFO(logger, "Compliance disallowed also in y direction");
        feedback->state = "Compliance disallowed also in y direction";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Hand guiding disallowed also in y direction");
} else if (msg == "DISALLOW COMPLIANCE ALSO Z") {
        if(!controller_->disallow_compliance_also("z")){
        RCLCPP_ERROR(logger, "Failed to disallow compliance also in z direction");
        feedback->state = "Failure while disallowing compliance also in z direction";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Failed to disallow hand guiding also in z direction");
        return;
    }
        RCLCPP_INFO(logger, "Compliance disallowed also in z direction");
        feedback->state = "Compliance disallowed also in z direction";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Hand guiding disallowed also in z direction");
} else if (msg == "DISALLOW COMPLIANCE ONLY X") {
        if(!controller_->disallow_compliance_only("x")){
        RCLCPP_ERROR(logger, "Failed to disallow compliance only in x direction");
        feedback->state = "Failure while disallowing compliance only in x direction";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Failed to disallow hand guiding only in x direction");
        return;
    }
        RCLCPP_INFO(logger, "Compliance disallowed only in x direction");
        feedback->state = "Compliance disallowed only in x direction";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Hand guiding disallowed only in x direction");
} else if (msg == "DISALLOW COMPLIANCE ONLY Y") {
        if(!controller_->disallow_compliance_only("y")){
        RCLCPP_ERROR(logger, "Failed to disallow compliance only in y direction");
        feedback->state = "Failure while disallowing compliance only in y direction";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Failed to disallow hand guiding only in y direction");
        return;
    }
        RCLCPP_INFO(logger, "Compliance disallowed only in y direction");
        feedback->state = "Compliance disallowed only in y direction";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Hand guiding disallowed only in y direction");
} else if (msg == "DISALLOW COMPLIANCE ONLY Z") {
        if(!controller_->disallow_compliance_only("z")){
        RCLCPP_ERROR(logger, "Failed to disallow compliance only in z direction");
        feedback->state = "Failure while disallowing compliance only in z direction";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Failed to disallow hand guiding only in z direction");
        return;
    }
        RCLCPP_INFO(logger, "Compliance disallowed only in z direction");
        feedback->state = "Compliance disallowed only in z direction";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Hand guiding disallowed only in z direction");

} else if (msg == "PICK PLY") {
    RCLCPP_INFO(logger, "Picking ply");
    feedback->state = "Picking ply";
    goal_handle->publish_feedback(feedback);
    tts_action_client_.send_tts_msg("Picking ply");

    controller_->pick_ply(
        [this, goal_handle, feedback](bool success)
        {
            if (!success) {
            feedback->state = "Failure while picking ply";
            goal_handle->publish_feedback(feedback);
            tts_action_client_.send_tts_msg("Failed to pick ply");
            return;
            }

            feedback->state = "Ply picked successfully";
            goal_handle->publish_feedback(feedback);
            tts_action_client_.send_tts_msg("Ply picked successfully");
        });

    } else if (msg == "STOP LISTENING") {

        RCLCPP_INFO(logger, "Voice command requested session end");

        std_msgs::msg::Empty end_msg;
        end_session_pub_->publish(end_msg);

        feedback->state = "Ending voice command session";
        goal_handle->publish_feedback(feedback);

        tts_action_client_.send_tts_msg("Stopping voice commands");

        result->success = true;
        goal_handle->succeed(result);
        return;
    } else if (msg == "END HAND GUIDING") {

        RCLCPP_INFO(logger, "Ending hand guiding requested through voice command");
        tts_action_client_.send_tts_msg("Ending hand guiding");

        if(!controller_->end_hand_guiding()){
            RCLCPP_ERROR(logger, "Failed to end hand guiding");
            feedback->state = "Failure while ending hand guiding";
            goal_handle->publish_feedback(feedback);
            tts_action_client_.send_tts_msg("Failed to end hand guiding");
            return;
    }
        RCLCPP_INFO(logger, "Hand guiding ended");
        feedback->state = "Hand guiding ended";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Hand guiding ended");
    } else if (msg == "MOVE CLOSE TO PICK") {
        RCLCPP_INFO(logger, "Moving close to pick");
        feedback->state = "Moving close to pick";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Moving close to pick");

        if (!controller_->move_close_to_pick()) {
            RCLCPP_ERROR(logger, "Failed to move close to pick");
            feedback->state = "Failure while moving close to pick";
            goal_handle->publish_feedback(feedback);
            tts_action_client_.send_tts_msg("Failed to move close to pick");
            return;
        }
        RCLCPP_INFO(logger, "Moved close to pick");
        feedback->state = "Moved close to pick";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Moved close to pick");
    } else if (msg == "APPROACH") {
        RCLCPP_INFO(logger, "Approaching the placing point");
        feedback->state = "Approaching the placing point";
        goal_handle->publish_feedback(feedback);
        tts_action_client_.send_tts_msg("Approaching the placing point");

        if (!controller_->approach_placing_point()) {
            RCLCPP_ERROR(logger, "Failed to approach the placing point");
            feedback->state = "Failure while approaching the placing point";
            goal_handle->publish_feedback(feedback);
            tts_action_client_.send_tts_msg("Failed to approach the placing point");
            return;
        }
        RCLCPP_INFO(logger, "Approached the placing point");
        feedback->state = "Approached the placing point";
        goal_handle->publish_feedback(feedback);
    }

    // End action
    result->success = true;
    goal_handle->succeed(result);
    return;
}