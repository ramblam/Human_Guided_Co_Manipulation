#ifndef RCM_MOVEIT_PAP_CONTROLLER_HPP
#define RCM_MOVEIT_PAP_CONTROLLER_HPP

#include <atomic>

#include "geometry_msgs/msg/pose.hpp"
#include "rcm_clients/cartesian_movement_client.hpp"
#include "rcm_clients/control_end_effector_client.hpp"
#include "rcm_clients/get_pose_client.hpp"

class PickAndPlaceController{
    public:
        /*
         * Params: node, controller
         *
         * Adds services through the node pointer, the callbacks of which use the controller pointer
         * to implement the responses.
         */
        PickAndPlaceController(std::shared_ptr<rclcpp::Node>& node);

        void set_force(double force);

        bool pick(const geometry_msgs::msg::Pose target_pose, double distance, bool advanced_mode = false);

        bool place(const geometry_msgs::msg::Pose starting_pose, double distance, bool advanced_mode = false);

    protected:
        bool approach(const geometry_msgs::msg::Pose target_pose, double distance, bool advanced_mode);

        bool retreat(const geometry_msgs::msg::Pose starting_pose, const geometry_msgs::msg::Pose target_pose, double distance, bool advanced_mode);

        geometry_msgs::msg::Pose translate_in_target_frame(const geometry_msgs::msg::Pose target_pose, double distance);

        geometry_msgs::msg::Pose compensate_ee_orientation(const geometry_msgs::msg::Pose target_pose, bool inverse = false);

        std::shared_ptr<rclcpp::Node> node_;
        
        std::vector<double> default_ee_orientation_;
        bool direct_ee_poses_; 
        std::vector<std::string> io_name_;
        std::vector<long int> io_state_;
        double width_ = 0.0;
        std::atomic<double> force_ = 0.5;
        double angle_ = 0.0;
        
        rcm_clients::ControlEndEffectorClient control_end_effector_client_;
        rcm_clients::GetPoseClient get_pose_client_;
        rcm_clients::CartesianMovementClient cartesian_movement_client_;
};

#endif