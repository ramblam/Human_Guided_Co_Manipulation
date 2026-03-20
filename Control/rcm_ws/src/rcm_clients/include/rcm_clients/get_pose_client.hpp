#ifndef RCM_GET_POSE_CLIENT_HPP
#define RCM_GET_POSE_CLIENT_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "rcm_msgs/srv/get_pose.hpp"

namespace rcm_clients {
  class GetPoseClient {
    public:
      /*
      * Params: node
      *
      * Creates a get pose service client through the node pointer
      */
      GetPoseClient(std::shared_ptr<rclcpp::Node>& node);

      /*
      * Params: pose object to be modified
      *
      * Returns: success
      * 
      * Creates a get pose service client through the node pointer
      */
      bool update_pose(geometry_msgs::msg::Pose &pose);

    protected:
      std::shared_ptr<rclcpp::Node> node_;
      rclcpp::Client<rcm_msgs::srv::GetPose>::SharedPtr get_pose_client_;
      std::string service_name_;
  };
} // namespace rcm_clients

#endif