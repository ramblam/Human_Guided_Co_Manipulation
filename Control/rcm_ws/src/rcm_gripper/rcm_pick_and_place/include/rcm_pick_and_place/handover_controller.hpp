#ifndef RCM_MOVEIT_HANDOVER_CONTROLLER_HPP
#define RCM_MOVEIT_HANDOVER_CONTROLLER_HPP

#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rcm_clients/control_end_effector_client.hpp"
#include "rcm_clients/get_pose_client.hpp"

namespace rcm_handover {
    class HandoverController{
        public:
            HandoverController(std::shared_ptr<rclcpp::Node>& node);

            bool compliant_handover(double displacement_threshold);

            void cancel_handover();
        
        protected:
            double euclidean_distance(geometry_msgs::msg::Pose pose0, 
                geometry_msgs::msg::Pose pose1);
            
            std::shared_ptr<rclcpp::Node> node_;
            rcm_clients::ControlEndEffectorClient control_end_effector_client_;
            rcm_clients::GetPoseClient get_pose_client_;

            std::atomic<bool> handover_canceled_;
            std::vector<double> default_ee_orientation_;
            std::vector<std::string> io_name_;
            std::vector<long int> io_state_;
            double width_ = 0.0;
            std::atomic<double> force_ = 0.5;
            double angle_ = 0.0;
    };
}

#endif