#include <thread>
#include <atomic>

#include "rcm_pick_and_place/pap_action.hpp"

PickAndPlaceAction::PickAndPlaceAction(std::shared_ptr<rclcpp::Node>& node, std::shared_ptr<PickAndPlaceController>& controller)
: node_(node), controller_(controller)
{
    using namespace std::placeholders;

    pick_and_place_server_ = rclcpp_action::create_server<PickAndPlace>(
      node_,
      "rcm_pick_and_place/pick_and_place",
      std::bind(&PickAndPlaceAction::pick_and_place_goal_received_cb, this, _1, _2),
      std::bind(&PickAndPlaceAction::pick_and_place_goal_cancelled_cb, this, _1),
      std::bind(&PickAndPlaceAction::pick_and_place_goal_accepted_cb, this, _1));

}

/* Implementation of Cartesian Trajectory Command Action:
 * - goal_received_cb()
 * - goal_cancelled_cb()
 * - goal_accepted_cb()
 * - execute()
 */

rclcpp_action::GoalResponse PickAndPlaceAction::pick_and_place_goal_received_cb( const rclcpp_action::GoalUUID & uuid,
                                                        std::shared_ptr<const PickAndPlace::Goal> goal){
    (void)uuid; // Unused parameter
    (void)goal; // Unused parameter

    auto logger = node_->get_logger();
    
    RCLCPP_INFO(logger, "Received pick and place goal");

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PickAndPlaceAction::pick_and_place_goal_cancelled_cb(
                                const std::shared_ptr<GoalHandlePickAndPlace> goal_handle){
    (void)goal_handle; // Unused parameter
    RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void PickAndPlaceAction::pick_and_place_goal_accepted_cb(const std::shared_ptr<GoalHandlePickAndPlace> goal_handle){
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&PickAndPlaceAction::pick_and_place_goal_execute, this, _1), goal_handle}.detach();                                
}


void PickAndPlaceAction::pick_and_place_goal_execute(const std::shared_ptr<GoalHandlePickAndPlace> goal_handle){

    auto logger = node_->get_logger();
    
    // Action setup
    RCLCPP_INFO(logger, "Processing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<PickAndPlace::Feedback>();
    auto result = std::make_shared<PickAndPlace::Result>();
    result->success = false;

    // Set grasping force
    controller_->set_force(goal->force);

    // Pick
    if(goal->pick){
      RCLCPP_INFO(logger, "Picking started");
      feedback->state = "Picking started";
      goal_handle->publish_feedback(feedback);

      if(!controller_->pick(goal->pick_pose, goal->distance, goal->advanced_mode)) {
        RCLCPP_ERROR(logger, "Failure during picking");
        feedback->state = "Failure during picking";
        goal_handle->publish_feedback(feedback);

        goal_handle->succeed(result);
        return;
      }
    }

    // Place
    if(goal->place){
      RCLCPP_INFO(logger, "Placing started");
      feedback->state = "Placing started";
      goal_handle->publish_feedback(feedback);

      if(!controller_->place(goal->place_pose, goal->distance, goal->advanced_mode)) {
        RCLCPP_ERROR(logger, "Failure during placing");
        feedback->state = "Failure during placing";
        goal_handle->publish_feedback(feedback);

        goal_handle->succeed(result);
        return;
      }
    }

    RCLCPP_INFO(logger, "Pick and place sequence completed");
    feedback->state = "Pick and place sequence completed";
    goal_handle->publish_feedback(feedback);

    // End action
    result->success = true;
    goal_handle->succeed(result);
    return;
}