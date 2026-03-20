#ifndef RCM_POSE_MOVEMENT_CLIENT_HPP
#define RCM_POSE_MOVEMENT_CLIENT_HPP
#include <future>

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rcm_msgs/action/pose_movement_command.hpp"

namespace rcm_clients {
  class PoseMovementClient {
    public:
      using PoseMovementCommand = rcm_msgs::action::PoseMovementCommand;
      using GoalHandlePoseMovementCommand = rclcpp_action::ClientGoalHandle<PoseMovementCommand>;

      PoseMovementClient(std::shared_ptr<rclcpp::Node>& node);
    
      bool execute_pose_movement(geometry_msgs::msg::Pose pose, double velocity=0.1, double timeout_ms=10000.0, std::string mode="ompl");

    protected:
      void pose_movement_response_callback(const GoalHandlePoseMovementCommand::SharedPtr & goal_handle);
      void pose_movement_feedback_callback(GoalHandlePoseMovementCommand::SharedPtr,
        const std::shared_ptr<const PoseMovementCommand::Feedback> feedback);
      void pose_movement_result_callback(const GoalHandlePoseMovementCommand::WrappedResult & result);

      std::shared_ptr<rclcpp::Node> node_;
      std::string action_name_;
      rclcpp_action::Client<PoseMovementCommand>::SendGoalOptions pose_movement_options_;
      rclcpp_action::Client<PoseMovementCommand>::SharedPtr pose_movement_client_;
  };
} // namespace rcm_clients

#endif