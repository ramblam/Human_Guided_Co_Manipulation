#ifndef RCM_CARTESIAN_MOVEMENT_CLIENT_HPP
#define RCM_CARTESIAN_MOVEMENT_CLIENT_HPP
#include <future>

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rcm_msgs/action/cartesian_trajectory_command.hpp"

namespace rcm_clients {
  class CartesianMovementClient {
    public:
      using CartesianTrajectoryCommand = rcm_msgs::action::CartesianTrajectoryCommand;
      using GoalHandleCartesianTrajectoryCommand = rclcpp_action::ClientGoalHandle<CartesianTrajectoryCommand>;

      CartesianMovementClient(std::shared_ptr<rclcpp::Node>& node);
    
      bool execute_cartesian_trajectory(std::vector<geometry_msgs::msg::Pose> poses, double velocity=0.1, double timeout_ms=10000.0);

    protected:
      void cartesian_movement_response_callback(const GoalHandleCartesianTrajectoryCommand::SharedPtr & goal_handle);
      void cartesian_movement_feedback_callback(GoalHandleCartesianTrajectoryCommand::SharedPtr,
        const std::shared_ptr<const CartesianTrajectoryCommand::Feedback> feedback);
      void cartesian_movement_result_callback(const GoalHandleCartesianTrajectoryCommand::WrappedResult & result);

      std::shared_ptr<rclcpp::Node> node_;
      std::string action_name_;
      rclcpp_action::Client<CartesianTrajectoryCommand>::SendGoalOptions cartesian_movement_options_;
      rclcpp_action::Client<CartesianTrajectoryCommand>::SharedPtr cartesian_movement_client_;
  };
} // namespace rcm_clients

#endif