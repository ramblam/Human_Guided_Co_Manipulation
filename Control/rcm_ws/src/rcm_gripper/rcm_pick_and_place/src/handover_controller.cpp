#include "rcm_pick_and_place/handover_controller.hpp"

#include <thread>
#include <tf2/LinearMath/Vector3.h>


namespace rcm_handover {

    HandoverController::HandoverController(std::shared_ptr<rclcpp::Node>& node)
    : control_end_effector_client_(node), get_pose_client_(node)
    {
        node_ = node;
        handover_canceled_ = false;
        io_name_ = {"gripper"};
        io_state_ = {0};
        width_ = 0.0;
        force_ = 0.5;
        angle_ = 0.0;
    }

    bool HandoverController::compliant_handover(double displacement_threshold){
        using namespace std::chrono_literals;

        auto logger = node_->get_logger();
        geometry_msgs::msg::Pose reference_pose;
        geometry_msgs::msg::Pose current_pose;

        if(!get_pose_client_.update_pose(reference_pose)){
            RCLCPP_ERROR(logger, "Failed to retrieve initial reference pose");
            return false;
        }

        handover_canceled_ = false;

        while(!handover_canceled_){
            if(handover_canceled_){
                RCLCPP_INFO(logger, "Handover canceled");
                return false;
            }
            else if(!get_pose_client_.update_pose(current_pose)){
                RCLCPP_ERROR(logger, "Failed to retrieve current pose");
                return false;
            }
            else if(euclidean_distance(reference_pose, current_pose) > displacement_threshold){
                io_state_[0] = 0;
                if(!control_end_effector_client_.control_end_effector(io_name_, io_state_, width_, force_, angle_)){
                    RCLCPP_ERROR(logger, "Releasing failed");
                    return false;
                }
                return true; 
            }
            else{
                std::this_thread::sleep_for(250ms);
            }
            
        }
        
        return false;
    }

    void HandoverController::cancel_handover(){
        handover_canceled_ = true;
    }

    double HandoverController::euclidean_distance(geometry_msgs::msg::Pose pose0, 
        geometry_msgs::msg::Pose pose1){
        
        tf2::Vector3 v0(
            pose0.position.x, 
            pose0.position.y, 
            pose0.position.z
        );

        tf2::Vector3 v1(
            pose1.position.x, 
            pose1.position.y, 
            pose1.position.z
        );

        double distance = v0.distance(v1);

        return distance;
    }

}