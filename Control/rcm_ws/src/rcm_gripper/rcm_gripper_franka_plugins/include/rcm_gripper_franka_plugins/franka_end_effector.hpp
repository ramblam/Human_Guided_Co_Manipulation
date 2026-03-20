#ifndef RCM_GRIPPERS_FRANKA_END_EFFECTOR_HPP
#define RCM_GRIPPERS_FRANKA_END_EFFECTOR_HPP
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "franka_msgs/action/grasp.hpp"
#include "franka_msgs/action/homing.hpp"
#include "franka_msgs/action/move.hpp"
#include "rcm_gripper/generic_end_effector_base.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace rcm_gripper_franka_plugins
{
  class FrankaEndEffector : public rcm_gripper::GenericEndEffector
  {
    /* Uses the action interfaces provided by franka_ros2 to implement a BasicGripper plugin
     * for Franka Hand (and other compatible grippers).
     */
    public:

      using Homing = franka_msgs::action::Homing;
      using Grasp = franka_msgs::action::Grasp;
      using Move = franka_msgs::action::Move;

      using HomingGoalHandle = rclcpp_action::ClientGoalHandle<Homing>;
      using GraspGoalHandle = rclcpp_action::ClientGoalHandle<Grasp>;
      using MoveGoalHandle = rclcpp_action::ClientGoalHandle<Move>;

      /* Must be called to initialzie the object after the plugin is loaded.
       *
       * Implementations should call init_base to initialize parent variables,
       * and may optionally call init_base_services and init_base_action. 
       */
      void init(std::shared_ptr<rclcpp::Node> node) override;
      
      bool move(double width) override;
      bool grasp(double force) override;
      bool release() override;

      bool homing();

    protected:
      bool stop();

      void end_effector_goal_execute(const std::shared_ptr<GoalHandleControlEndEffector> goal_handle) override;

      void send_feedback(const std::shared_ptr<GoalHandleControlEndEffector> goal_handle,
        std::string msg, std::string msg_style);

      // franka_gripper action client
      void init_cli();

      void homing_response_callback(const HomingGoalHandle::SharedPtr & goal_handle);
      void homing_feedback_callback(HomingGoalHandle::SharedPtr,
        const std::shared_ptr<const Homing::Feedback> feedback);
      void homing_result_callback(const HomingGoalHandle::WrappedResult & result);

      void grasp_response_callback(const GraspGoalHandle::SharedPtr & goal_handle);
      void grasp_feedback_callback(GraspGoalHandle::SharedPtr,
        const std::shared_ptr<const Grasp::Feedback> feedback);
      void grasp_result_callback(const GraspGoalHandle::WrappedResult & result);

      void move_response_callback(const MoveGoalHandle::SharedPtr & goal_handle);
      void move_feedback_callback(MoveGoalHandle::SharedPtr,
        const std::shared_ptr<const Move::Feedback> feedback);
      void move_result_callback(const MoveGoalHandle::WrappedResult & result);
      
      rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;

      rclcpp_action::Client<Homing>::SendGoalOptions homing_options_;
      rclcpp_action::Client<Homing>::SharedPtr homing_client_;

      rclcpp_action::Client<Grasp>::SendGoalOptions grasp_options_;
      rclcpp_action::Client<Grasp>::SharedPtr grasp_client_;

      rclcpp_action::Client<Move>::SendGoalOptions move_options_;
      rclcpp_action::Client<Move>::SharedPtr move_client_;

      double max_width_;
      double speed_ = 0.1;
  };
} // namespace rcm_gripper_franka_plugins

#endif