#ifndef RCM_MANAGER_BASE_HPP
#define RCM_MANAGER_BASE_HPP

#include <thread>
#include <atomic>
#include <future>

#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include "rcm_msgs/srv/set_stiffness.hpp"
#include "rcm_msgs/srv/set_speed.hpp"

namespace rcm_manager
{
  class ManagerBase
  {
    public:
      void init_base(std::shared_ptr<rclcpp::Node> node);
      virtual void init(std::shared_ptr<rclcpp::Node> node) = 0;
      virtual ~ManagerBase(){}

    protected:
      ManagerBase(){}

      static std::string str_vec_to_str(std::vector<std::string>);

      void async_switch_controller(const std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service,
        const std::shared_ptr<rmw_request_id_t> request_header,
        std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::vector<std::string> activate_controllers);

      std::vector<std::string> get_controllers_to_deactivate(std::vector<std::string> activate_controllers); 

      double scale_stiffness(double value, double min, double max);

      // Using deferred service callback format to allow responses to depend on other services
      void load_compliant_controller_cb(const std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service,
        const std::shared_ptr<rmw_request_id_t> request_header,
        std::shared_ptr<std_srvs::srv::Trigger::Request> request);

      void load_compliant_teleop_controller_cb(const std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service,
        const std::shared_ptr<rmw_request_id_t> request_header,
        std::shared_ptr<std_srvs::srv::Trigger::Request> request);

      void load_default_controller_cb(const std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service,
        const std::shared_ptr<rmw_request_id_t> request_header,
        std::shared_ptr<std_srvs::srv::Trigger::Request> request);
      
      void load_default_teleop_controller_cb(const std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service,
        const std::shared_ptr<rmw_request_id_t> request_header,
        std::shared_ptr<std_srvs::srv::Trigger::Request> request);
      
      void set_speed_cb(const std::shared_ptr<rclcpp::Service<rcm_msgs::srv::SetSpeed>> service,
        const std::shared_ptr<rmw_request_id_t> request_header,
        std::shared_ptr<rcm_msgs::srv::SetSpeed::Request> request);

      virtual void set_stiffness_cb(const std::shared_ptr<rclcpp::Service<rcm_msgs::srv::SetStiffness>> service,
        const std::shared_ptr<rmw_request_id_t> request_header,
        std::shared_ptr<rcm_msgs::srv::SetStiffness::Request> request) = 0;
      
      std::shared_ptr<rclcpp::Node> node_;
      std::vector<std::string> managed_controllers_;
      std::vector<std::string> compliant_controllers_;
      std::vector<std::string> compliant_teleop_controllers_;
      std::vector<std::string> default_controllers_;
      std::vector<std::string> default_teleop_controllers_;
      std::string stiffness_param_service_;

      double trans_stiffness_max_;
      double trans_stiffness_min_;
      double rot_stiffness_max_;
      double rot_stiffness_min_;

      rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_compliant_controller_service_;
      rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_compliant_teleop_controller_service_;
      rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_default_controller_service_;
      rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_default_teleop_controller_service_;
      rclcpp::Service<rcm_msgs::srv::SetSpeed>::SharedPtr set_speed_service_;
      rclcpp::Service<rcm_msgs::srv::SetStiffness>::SharedPtr set_stiffness_service_;

      rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;
      rclcpp::Client<rcm_msgs::srv::SetSpeed>::SharedPtr set_speed_client_;

  };
} // namespace rcm_manager

#endif