#ifndef RCM_MANAGER_CIC_PLUGINS_HPP
#define RCM_MANAGER_CIC_PLUGINS_HPP

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include "rcm_manager/manager_base.hpp"

namespace rcm_manager_cic_plugins
{
  class CartesianImpedanceControllerManager : public rcm_manager::ManagerBase
  {
    public:
      void init(std::shared_ptr<rclcpp::Node> node) override;
    
    protected:
      void set_stiffness_cb(const std::shared_ptr<rclcpp::Service<rcm_msgs::srv::SetStiffness>> service,
        const std::shared_ptr<rmw_request_id_t> request_header,
        std::shared_ptr<rcm_msgs::srv::SetStiffness::Request> request) override;

        rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr set_cic_parameters_client_;
  };
} // namespace rcm_manager_cic_plugins

#endif
