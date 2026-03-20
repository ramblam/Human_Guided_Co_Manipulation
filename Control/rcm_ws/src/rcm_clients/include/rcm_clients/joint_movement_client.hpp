#ifndef RCM_JOINT_MOVEMENT_CLIENT_HPP
#define RCM_JOINT_MOVEMENT_CLIENT_HPP
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rcm_msgs/action/joint_movement_command.hpp"

namespace rcm_clients {
  class JointMovementClient {
    public:
      using JointMovementCommand = rcm_msgs::action::JointMovementCommand;
      using GoalHandleJointMovementCommand = rclcpp_action::ClientGoalHandle<JointMovementCommand>;

      JointMovementClient(std::shared_ptr<rclcpp::Node>& node);
    
      bool execute_joint_movement(std::vector<double> joint_positions, double velocity=0.1, double acceleration=0.1, double timeout_ms=10000.0);

    protected:
      void joint_movement_response_callback(const GoalHandleJointMovementCommand::SharedPtr & goal_handle);
      void joint_movement_feedback_callback(GoalHandleJointMovementCommand::SharedPtr,
        const std::shared_ptr<const JointMovementCommand::Feedback> feedback);
      void joint_movement_result_callback(const GoalHandleJointMovementCommand::WrappedResult & result);

      std::shared_ptr<rclcpp::Node> node_;
      std::string action_name_;
      rclcpp_action::Client<JointMovementCommand>::SendGoalOptions joint_movement_options_;
      rclcpp_action::Client<JointMovementCommand>::SharedPtr joint_movement_client_;
  };
}

#endif