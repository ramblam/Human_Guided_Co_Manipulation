#ifndef RCM_GRIPPER_GENERIC_END_EFFECTOR_BASE_HPP
#define RCM_GRIPPER_GENERIC_END_EFFECTOR_BASE_HPP

#include <atomic>
#include <string>
#include <thread>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "rcm_msgs/action/control_end_effector.hpp"

namespace rcm_gripper
{
  class GenericEndEffector
  {
    public:
      using ControlEndEffector = rcm_msgs::action::ControlEndEffector;
      using GoalHandleControlEndEffector = rclcpp_action::ServerGoalHandle<ControlEndEffector>;

      /* Must be called to initialzie the object after the plugin is loaded.
       *
       * Implementations should call init_base to initialize parent variables,
       * and may optionally call init_base_services and init_base_action. 
       */
      virtual void init(std::shared_ptr<rclcpp::Node> node) = 0;
      
      /* Moves the gripper fingers to <width> (m).
       *
       * If not applicable, implement a suitable warning message.
       */
      virtual bool move(double width) = 0;
      /* Grasp with <force> (Nm). Required for init_base_services.
       *
       * If not applicable, implement a suitable warning message.
       */
      virtual bool grasp(double force) = 0;
      /* Releases a grasped object. Required for init_base_services.
       *
       * If not applicable, implement a suitable warning message.
       */
      virtual bool release() = 0;

      virtual ~GenericEndEffector(){}

      void init_base_services();

      void init_base_action();

      rclcpp_action::Server<ControlEndEffector>::SharedPtr end_effector_cmd_server_;

    protected:
      GenericEndEffector(){}
      
      void init_base(std::shared_ptr<rclcpp::Node> node);

      void close_hand_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

      void open_hand_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
      
      rclcpp_action::GoalResponse end_effector_goal_received_cb( const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ControlEndEffector::Goal> goal);

      rclcpp_action::CancelResponse end_effector_goal_cancelled_cb(
      const std::shared_ptr<GoalHandleControlEndEffector> goal_handle);

      void end_effector_goal_accepted_cb(const std::shared_ptr<GoalHandleControlEndEffector> goal_handle);
      
      /* Executes the goal received by the action server.
       *
       * If necessary, goals and cancellation requests can be rejected by setting
       * goal_can_be_accepted_ and/or goal_can_be_cancelled_ to false.
       */
      virtual void end_effector_goal_execute(const std::shared_ptr<GoalHandleControlEndEffector> goal_handle) = 0;

      std::shared_ptr<rclcpp::Node> node_;

      std::atomic<bool> goal_can_be_accepted_ = true;
      std::atomic<bool> goal_can_be_cancelled_ = true;

      rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr close_service_;
      rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr open_service_;

      rclcpp::CallbackGroup::SharedPtr base_service_callback_group_;
  };
} // namespace rcm_gripper

#endif