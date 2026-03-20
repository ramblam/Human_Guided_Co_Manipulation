#include "rcm_clients/get_joint_states_client.hpp"


namespace rcm_clients {
  
  GetJointsClient::GetJointsClient(std::shared_ptr<rclcpp::Node>& node){
    node_ = node;
    service_name_ = "rcm_moveit/get_joints";
    get_joints_client_ = node_->create_client<rcm_msgs::srv::GetJoints>(service_name_);
  }

  bool GetJointsClient::update_joints(std::vector<double> &joint_positions){
    using namespace std::chrono_literals;

    auto logger = node_->get_logger();
    
    auto get_joints_request = std::make_shared<rcm_msgs::srv::GetJoints::Request>();
    int n = 0;
    bool success = true;

    while (!get_joints_client_->wait_for_service(1s) && n < 10) {
      ++n;
      RCLCPP_WARN(logger, "Service %s not available, trying again...", service_name_.c_str());
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(logger, "Interrupted while waiting for %s. Exiting.", service_name_.c_str());
        success = false;
        break;
      }
      else if(n>9) {
        RCLCPP_ERROR(logger, "Timeout while waiting for %s. Exiting.", service_name_.c_str());
        success = false;
      }
    }

    if(success){
      std::future_status status;
      auto result = get_joints_client_->async_send_request(get_joints_request);

      switch (status = result.wait_for(5s); status){
        case std::future_status::deferred:
          RCLCPP_INFO(logger, "Did not receive a response from %s: deferred", service_name_.c_str());
          success = false;
          break;

        case std::future_status::timeout:
          success = false;
          RCLCPP_INFO(logger, "Did not receive a response from %s: timeout", service_name_.c_str());
          break;

        case std::future_status::ready:
          joint_positions = result.get()->joint_positions;
          RCLCPP_INFO(logger, "Received response from %s", service_name_.c_str());
          break;
      }
    }

    return success;
  }
} // namespace rcm_clients