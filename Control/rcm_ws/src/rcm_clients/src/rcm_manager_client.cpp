#include "rcm_clients/rcm_manager_client.hpp"


namespace rcm_clients {
  
  RCMManagerClient::RCMManagerClient(std::shared_ptr<rclcpp::Node>& node){
    node_ = node;

    load_default_controller_service_name_ = "/rcm_manager/load_default_controller";
    load_compliant_controller_service_name_ = "/rcm_manager/load_compliant_controller"; 
    set_stiffness_service_name_ = "/rcm_manager/set_stiffness";

    load_default_controller_client_ = node_->create_client<std_srvs::srv::Trigger>(load_default_controller_service_name_);
    load_compliant_controller_client_ = node_->create_client<std_srvs::srv::Trigger>(load_compliant_controller_service_name_);
    set_stiffness_client_ = node_->create_client<rcm_msgs::srv::SetStiffness>(set_stiffness_service_name_);
  }

  bool RCMManagerClient::load_default_controller(){
    using namespace std::chrono_literals;

    auto logger = node_->get_logger();
    
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    int n = 0;
    bool success = true;

    while (!load_default_controller_client_->wait_for_service(1s) && n < 10) {
      ++n;
      RCLCPP_WARN(logger, "Service %s not available, trying again...", load_default_controller_service_name_.c_str());
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(logger, "Interrupted while waiting for %s. Exiting.", load_default_controller_service_name_.c_str());
        success = false;
        break;
      }
      else if(n>9) {
        RCLCPP_ERROR(logger, "Timeout while waiting for %s. Exiting.", load_default_controller_service_name_.c_str());
        success = false;
      }
    }

    if(success){
      std::future_status status;
      auto result_future = load_default_controller_client_->async_send_request(request);

      switch (status = result_future.wait_for(5s); status){
        case std::future_status::deferred:
          RCLCPP_INFO(logger, "Did not receive a response from %s: deferred", load_default_controller_service_name_.c_str());
          success = false;
          break;

        case std::future_status::timeout:
          success = false;
          RCLCPP_INFO(logger, "Did not receive a response from %s: timeout", load_default_controller_service_name_.c_str());
          break;

        case std::future_status::ready:
          auto result = result_future.get();
          std::string message = result->message;
          success = result->success;
          RCLCPP_INFO(logger, "Received response from %s: '%s'", load_default_controller_service_name_.c_str(), message.c_str());
          break;
      }
    }

    return success;
  }

  bool RCMManagerClient::load_compliant_controller(){
    using namespace std::chrono_literals;

    auto logger = node_->get_logger();
    
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    int n = 0;
    bool success = true;

    while (!load_compliant_controller_client_->wait_for_service(1s) && n < 10) {
      ++n;
      RCLCPP_WARN(logger, "Service %s not available, trying again...", load_compliant_controller_service_name_.c_str());
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(logger, "Interrupted while waiting for %s. Exiting.", load_compliant_controller_service_name_.c_str());
        success = false;
        break;
      }
      else if(n>9) {
        RCLCPP_ERROR(logger, "Timeout while waiting for %s. Exiting.", load_compliant_controller_service_name_.c_str());
        success = false;
      }
    }

    if(success){
      std::future_status status;
      auto result_future = load_compliant_controller_client_->async_send_request(request);

      switch (status = result_future.wait_for(5s); status){
        case std::future_status::deferred:
          RCLCPP_INFO(logger, "Did not receive a response from %s: deferred", load_compliant_controller_service_name_.c_str());
          success = false;
          break;

        case std::future_status::timeout:
          success = false;
          RCLCPP_INFO(logger, "Did not receive a response from %s: timeout", load_compliant_controller_service_name_.c_str());
          break;

        case std::future_status::ready:
          auto result = result_future.get();
          std::string message = result->message;
          success = result->success;
          RCLCPP_INFO(logger, "Received response from %s: '%s'", load_compliant_controller_service_name_.c_str(), message.c_str());
          break;
      }
    }

    return success;
  }

  bool RCMManagerClient::set_stiffness(std::vector<double> stiffness){
    using namespace std::chrono_literals;

    auto logger = node_->get_logger();

    if(stiffness.size() != 6){
      RCLCPP_ERROR(logger, "Stiffness vector length is %ld, should be 6. Exiting.", stiffness.size());
    }
    
    auto request = std::make_shared<rcm_msgs::srv::SetStiffness::Request>();

    request->stiffness.force.x = stiffness[0];
    request->stiffness.force.y = stiffness[1];
    request->stiffness.force.z = stiffness[2];
    request->stiffness.torque.x = stiffness[3];
    request->stiffness.torque.y = stiffness[4];
    request->stiffness.torque.z = stiffness[5];

    int n = 0;
    bool success = true;

    while (!set_stiffness_client_->wait_for_service(1s) && n < 10) {
      ++n;
      RCLCPP_WARN(logger, "Service %s not available, trying again...", set_stiffness_service_name_.c_str());
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(logger, "Interrupted while waiting for %s. Exiting.", set_stiffness_service_name_.c_str());
        success = false;
        break;
      }
      else if(n>9) {
        RCLCPP_ERROR(logger, "Timeout while waiting for %s. Exiting.", set_stiffness_service_name_.c_str());
        success = false;
      }
    }

    if(success){
      std::future_status status;
      auto result_future = set_stiffness_client_->async_send_request(request);

      switch (status = result_future.wait_for(5s); status){
        case std::future_status::deferred:
          RCLCPP_INFO(logger, "Did not receive a response from %s: deferred", set_stiffness_service_name_.c_str());
          success = false;
          break;

        case std::future_status::timeout:
          success = false;
          RCLCPP_INFO(logger, "Did not receive a response from %s: timeout", set_stiffness_service_name_.c_str());
          break;

        case std::future_status::ready:
          auto result = result_future.get();
          std::string message = result->message;
          success = result->success;
          RCLCPP_INFO(logger, "Received response from %s: '%s'", set_stiffness_service_name_.c_str(), message.c_str());
          break;
      }
    }

    return success;
  }

} // namespace rcm_clients