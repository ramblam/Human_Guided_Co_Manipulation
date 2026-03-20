#ifndef RCM_MOVEIT_PAP_SERVICES_HPP
#define RCM_MOVEIT_PAP_SERVICES_HPP

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcm_pick_and_place/pap_controller.hpp"
#include "rcm_msgs/srv/target.hpp"

/*
 * A service wrapper for the RCM Moveit Controller
 *
 * Adds services to a node object, with callbacks using a MoveItController object.
 */
class PickAndPlaceServices {
    public:
        /*
         * Params: node, controller
         *
         * Adds services through the node pointer, the callbacks of which use the controller pointer
         * to implement the responses.
         */
        PickAndPlaceServices(std::shared_ptr<rclcpp::Node>& node, 
            std::shared_ptr<PickAndPlaceController>& controller);

    private:
        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<PickAndPlaceController> controller_;
        
        rclcpp::Service<rcm_msgs::srv::Target>::SharedPtr pick_service_;
        rclcpp::Service<rcm_msgs::srv::Target>::SharedPtr place_service_;

        void pick_cb(const std::shared_ptr<rcm_msgs::srv::Target::Request> request,
                        std::shared_ptr<rcm_msgs::srv::Target::Response> response);

        void place_cb(const std::shared_ptr<rcm_msgs::srv::Target::Request> request,
                        std::shared_ptr<rcm_msgs::srv::Target::Response> response);
};

#endif