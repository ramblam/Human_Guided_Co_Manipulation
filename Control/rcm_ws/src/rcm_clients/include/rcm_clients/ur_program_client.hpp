#ifndef RCM_UR_PROGRAM_CLIENT_HPP
#define RCM_UR_PROGRAM_CLIENT_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "ur_dashboard_msgs/srv/load.hpp"

namespace rcm_clients {

  class URProgramClient {
    public:
      /*
      * Params: node
      *
      * Creates a get joints service client through the node pointer
      */
      URProgramClient(std::shared_ptr<rclcpp::Node>& node);

      /*
      * Params: joints vector to be modified
      *
      * Returns: success
      * 
      * Creates a get joints service client through the node pointer
      */
      bool load_program(std::string filename);

      bool play_program();

      bool load_and_play_program(std::string filename);

    protected:

      std::shared_ptr<rclcpp::Node> node_;
      rclcpp::Client<ur_dashboard_msgs::srv::Load>::SharedPtr load_client_;
      rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr play_client_;
      std::string load_service_name_;
      std::string play_service_name_;  
  };

} // namespace rcm_clients

#endif