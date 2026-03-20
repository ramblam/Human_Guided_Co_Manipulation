#ifndef RCM_UR_PROGRAM_CLIENT_HPP
#define RCM_UR_PROGRAM_CLIENT_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "rcm_msgs/srv/set_stiffness.hpp"

namespace rcm_clients {

  class RCMManagerClient {
    public:
      /*
      * Params: node
      *
      * Creates service clients to interact with RCM Manager through the node pointer
      */
      RCMManagerClient(std::shared_ptr<rclcpp::Node>& node);

      /*
      * Params: 
      *
      * Attempts to load preconfigured default controllers, returns success
      */
      bool load_default_controller();

      /*
      * Params:
      *
      * Attempts to load preconfigured default controllers, returns success
      */
      bool load_compliant_controller();

      /*
      * Params: stiffness {x, y, z, rx, ry, rz}
      *
      * Attempts to set stiffness values through the RCM Manager. These stiffness values should be
      * normalized to [0.0, 1.0], and will be automatically scaled to different controllers.
      */
      bool set_stiffness(std::vector<double> stiffness);

    protected:

      std::shared_ptr<rclcpp::Node> node_;
      rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr load_default_controller_client_;
      rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr load_compliant_controller_client_;
      rclcpp::Client<rcm_msgs::srv::SetStiffness>::SharedPtr set_stiffness_client_; 

      std::string load_default_controller_service_name_;
      std::string load_compliant_controller_service_name_; 
      std::string set_stiffness_service_name_;
  };

} // namespace rcm_clients

#endif