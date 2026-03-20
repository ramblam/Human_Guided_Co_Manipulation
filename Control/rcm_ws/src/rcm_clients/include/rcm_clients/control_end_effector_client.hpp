#ifndef RCM_CONTROL_END_EFFECTOR_CLIENT_HPP
#define RCM_CONTROL_END_EFFECTOR_CLIENT_HPP
#include <future>

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rcm_msgs/action/control_end_effector.hpp"

namespace rcm_clients {
  class ControlEndEffectorClient {
    public:
      using ControlEndEffector = rcm_msgs::action::ControlEndEffector;
      using GoalHandleControlEndEffector = rclcpp_action::ClientGoalHandle<ControlEndEffector>;

      ControlEndEffectorClient(std::shared_ptr<rclcpp::Node>& node);
    
      bool control_end_effector(std::vector<std::string> io_name, std::vector<long int> io_state,
        double width, double force, double angle);

    protected:
      void control_end_effector_response_callback(const GoalHandleControlEndEffector::SharedPtr & goal_handle);
      void control_end_effector_feedback_callback(GoalHandleControlEndEffector::SharedPtr,
        const std::shared_ptr<const ControlEndEffector::Feedback> feedback);
      void control_end_effector_result_callback(const GoalHandleControlEndEffector::WrappedResult & result);

      std::shared_ptr<rclcpp::Node> node_;
      std::string action_name_;
      rclcpp_action::Client<ControlEndEffector>::SendGoalOptions control_end_effector_options_;
      rclcpp_action::Client<ControlEndEffector>::SharedPtr control_end_effector_client_;
  };
} // namespace rcm_clients

#endif