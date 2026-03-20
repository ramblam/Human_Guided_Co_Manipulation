#ifndef RCM_GRIPPER_GRIPPER_BASE_HPP
#define RCM_GRIPPER_GRIPPER_BASE_HPP

#include <thread>
#include <future>

#include <rclcpp/rclcpp.hpp>
#include "std_srvs/srv/trigger.hpp"

namespace rcm_gripper
{
  class BasicGripper 
  {
    public:
      virtual void init(std::shared_ptr<rclcpp::Node> node) = 0;
      virtual bool open() = 0;
      virtual bool close() = 0;
      virtual bool set_force(double force) = 0;
      virtual ~BasicGripper(){}
      
      void init_base_services();

    protected:
      BasicGripper(){}

      void close_hand_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

      void open_hand_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
      
      std::shared_ptr<rclcpp::Node> node_;
      rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr close_service_;
      rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr open_service_;

      rclcpp::CallbackGroup::SharedPtr base_service_callback_group_;
  };
} // namespace rcm_gripper

#endif