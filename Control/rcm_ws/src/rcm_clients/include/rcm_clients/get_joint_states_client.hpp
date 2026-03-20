#ifndef RCM_GET_JOINTS_CLIENT_HPP
#define RCM_GET_JOINTS_CLIENT_HPP

#include "rclcpp/rclcpp.hpp"
#include "rcm_msgs/srv/get_joints.hpp"

namespace rcm_clients {

  class GetJointsClient {
    public:
      /*
      * Params: node
      *
      * Creates a get joints service client through the node pointer
      */
      GetJointsClient(std::shared_ptr<rclcpp::Node>& node);

      /*
      * Params: joints vector to be modified
      *
      * Returns: success
      * 
      * Creates a get joints service client through the node pointer
      */
      bool update_joints(std::vector<double> &joint_positions);

    protected:
      std::shared_ptr<rclcpp::Node> node_;
      rclcpp::Client<rcm_msgs::srv::GetJoints>::SharedPtr get_joints_client_;
      std::string service_name_;
  };

} // namespace rcm_clients

#endif