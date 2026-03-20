#ifndef RCM_PAP_MOVEMENT_ACTION_
#define RCM_PAP_MOVEMENT_ACTION_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rcm_pick_and_place/pap_controller.hpp"
#include "rcm_msgs/action/pick_and_place.hpp"

class PickAndPlaceAction {
    public:
        using PickAndPlace = rcm_msgs::action::PickAndPlace;
        using GoalHandlePickAndPlace = rclcpp_action::ServerGoalHandle<PickAndPlace>;

        /*
         * Params: node, controller
         *
         * Adds actions through the node pointer, the callbacks of which use the controller pointer
         * to implement the responses.
         */
        PickAndPlaceAction(std::shared_ptr<rclcpp::Node>& node, std::shared_ptr<PickAndPlaceController>& controller);

        rclcpp_action::Server<PickAndPlace>::SharedPtr pick_and_place_server_;

    private:
        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<PickAndPlaceController> controller_;

        rclcpp_action::GoalResponse pick_and_place_goal_received_cb( const rclcpp_action::GoalUUID & uuid,
                                                        std::shared_ptr<const PickAndPlace::Goal> goal);
        rclcpp_action::CancelResponse pick_and_place_goal_cancelled_cb(
                                        const std::shared_ptr<GoalHandlePickAndPlace> goal_handle);
        
        void pick_and_place_goal_accepted_cb(const std::shared_ptr<GoalHandlePickAndPlace> goal_handle);

        void pick_and_place_goal_feedback(const std::shared_ptr<GoalHandlePickAndPlace> goal_handle);
        
        void pick_and_place_goal_execute(const std::shared_ptr<GoalHandlePickAndPlace> goal_handle);

};

#endif