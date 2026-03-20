#ifndef RCM_PICK_AND_PLACE_CLIENT_HPP
#define RCM_PICK_AND_PLACE_CLIENT_HPP
#include <future>

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rcm_msgs/action/pick_and_place.hpp"

namespace rcm_clients {
  class PickAndPlaceClient {
    public:
      using PickAndPlace = rcm_msgs::action::PickAndPlace;
      using GoalHandlePickAndPlace = rclcpp_action::ClientGoalHandle<PickAndPlace>;

      PickAndPlaceClient(std::shared_ptr<rclcpp::Node>& node);
    
      bool pick_and_place(geometry_msgs::msg::Pose pick_pose, 
        geometry_msgs::msg::Pose place_pose, double distance, double force, bool advanced_mode);
      
      bool pick(geometry_msgs::msg::Pose pick_pose, double distance, double force, bool advanced_mode=false);
   
      bool place(geometry_msgs::msg::Pose place_pose, double distance, double force, bool advanced_mode=false);
   
    protected:
      bool wait_for_result(PickAndPlace::Goal goal_msg);
      void pick_and_place_response_callback(const GoalHandlePickAndPlace::SharedPtr & goal_handle);
      void pick_and_place_feedback_callback(GoalHandlePickAndPlace::SharedPtr,
        const std::shared_ptr<const PickAndPlace::Feedback> feedback);
      void pick_and_place_result_callback(const GoalHandlePickAndPlace::WrappedResult & result);

      std::shared_ptr<rclcpp::Node> node_;
      std::string action_name_;
      rclcpp_action::Client<PickAndPlace>::SendGoalOptions pick_and_place_options_;
      rclcpp_action::Client<PickAndPlace>::SharedPtr pick_and_place_client_;
  };
} // namespace rcm_clients

#endif