#include <functional>

#include "rcm_manager/manager_base.hpp"

namespace rcm_manager
{
  void ManagerBase::init_base(std::shared_ptr<rclcpp::Node> node){
    using namespace std::placeholders;

    node_ = node;

    auto logger = node_->get_logger();

    // Init params
    std::vector<std::string> default_controllers = {"fr3_arm_controller"};
    node_->declare_parameter("default_controllers", default_controllers);
    node_->get_parameter("default_controllers", default_controllers_);
    RCLCPP_INFO(logger, "Loaded param: default_controllers:=%s", str_vec_to_str(default_controllers_).c_str());

    std::vector<std::string> compliant_controllers = {"fr3_arm_compliant_controller"};
    node_->declare_parameter("compliant_controllers", compliant_controllers);
    node_->get_parameter("compliant_controllers", compliant_controllers_);
    RCLCPP_INFO(logger, "Loaded param: compliant_controllers:=%s", str_vec_to_str(compliant_controllers_).c_str());

    std::string stiffness_param_service = "fr3_arm_compliant_controller/set_parameters_atomically";
    node_->declare_parameter("stiffness_param_service", stiffness_param_service);
    node_->get_parameter("stiffness_param_service", stiffness_param_service_);
    RCLCPP_INFO(logger, "Loaded param: stiffness_param_service:=\"%s\"", stiffness_param_service_.c_str());

    double trans_stiffness_max = 1500.0;
    node_->declare_parameter("trans_stiffness_max", trans_stiffness_max);
    node_->get_parameter("trans_stiffness_max", trans_stiffness_max_);
    RCLCPP_INFO(logger, "Loaded param: trans_stiffness_max:=%f", trans_stiffness_max_);

    double trans_stiffness_min = 0.0;
    node_->declare_parameter("trans_stiffness_min", trans_stiffness_min);
    node_->get_parameter("trans_stiffness_min", trans_stiffness_min_);
    RCLCPP_INFO(logger, "Loaded param: trans_stiffness_min:=%f", trans_stiffness_min_);

    double rot_stiffness_max = 100.0;
    node_->declare_parameter("rot_stiffness_max", rot_stiffness_max);
    node_->get_parameter("rot_stiffness_max", rot_stiffness_max_);
    RCLCPP_INFO(logger, "Loaded param: rot_stiffness_max:=%f", rot_stiffness_max_);

    double rot_stiffness_min = 0.0;
    node_->declare_parameter("rot_stiffness_min", rot_stiffness_min);
    node_->get_parameter("rot_stiffness_min", rot_stiffness_min_);
    RCLCPP_INFO(logger, "Loaded param: rot_stiffness_min:=%f", rot_stiffness_min_);

    // Init services
    main_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    load_compliant_controller_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "rcm_manager/load_compliant_controller", 
      std::bind(&ManagerBase::load_compliant_controller_cb, this, _1, _2),
      rclcpp::SystemDefaultsQoS(),
      main_callback_group_);
    
    load_default_controller_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "rcm_manager/load_default_controller", 
      std::bind(&ManagerBase::load_default_controller_cb, this, _1, _2),
      rclcpp::SystemDefaultsQoS(),
      main_callback_group_);

    set_stiffness_service_ = node_->create_service<rcm_msgs::srv::SetStiffness>(
      "rcm_manager/set_stiffness", 
      std::bind(&ManagerBase::set_stiffness_cb, this, _1, _2, _3),
      rclcpp::SystemDefaultsQoS(),
      main_callback_group_);

    switch_controller_client_ = node_->create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller",
      rclcpp::SystemDefaultsQoS(),
      client_callback_group_);

    RCLCPP_INFO(logger, "Manager services created");
  }

  std::string ManagerBase::str_vec_to_str(std::vector<std::string> vec){
    std::stringstream vec_stream;
    vec_stream << "[";

    for(auto it = vec.begin(); it != vec.end(); ++it){
      if(it != vec.begin()){
        vec_stream <<  ", ";
      }

      vec_stream << "\"" << *it << "\"";
    }

    vec_stream << "]";

    return vec_stream.str();
  }

  void ManagerBase::switch_controller(std::vector<std::string> activate_controllers,
    std::vector<std::string> deactivate_controllers,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    using namespace std::chrono_literals;

    (void)request;

    auto logger = node_->get_logger();

    auto request_inner = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request_inner->activate_controllers = activate_controllers;
    request_inner->deactivate_controllers = deactivate_controllers;
    request_inner->strictness = 1;

    RCLCPP_INFO(logger, "Connecting to /controller_manager/switch_controller");

    bool service_ready = true;

    response->message = "Unknown";
    response->success = false;

    int n = 0;

    while (!switch_controller_client_->wait_for_service(1s) && n < 5) {
      ++n;
      RCLCPP_WARN(logger, "Service /controller_manager/switch_controller not available, trying again...");
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(logger, "Interrupted while waiting for /controller_manager/switch_controller. Exiting.");
        service_ready = false;
        response->message = "Interrupted while waiting for /controller_manager/switch_controller";
        response->success = false;
        break;
      }
      else if(n>9) {
        RCLCPP_ERROR(logger, "Timeout while waiting for /controller_manager/switch_controller. Exiting.");
        service_ready = false;
        response->message = "Service /controller_manager/switch_controller not ready";
        response->success = false;
      }
    }

    if(service_ready){
      std::future_status status;
      auto result_future = switch_controller_client_->async_send_request(request_inner);

      switch (status = result_future.wait_for(5s); status){
        case std::future_status::deferred:
          RCLCPP_INFO(logger, "Service call to /controller_manager/switch_controller failed : deferred");
          response->message = "Service call to /controller_manager/switch_controller failed : deferred";
          response->success = false;
          break;

        case std::future_status::timeout:
          RCLCPP_INFO(logger, "Service call to /controller_manager/switch_controller failed : timeout");
          response->message = "Service call to /controller_manager/switch_controller failed : timeout";
          response->success = false;
          break;

        case std::future_status::ready:
          auto result = result_future.get();
          RCLCPP_INFO(logger, "Received response from /controller_manager/switch_controller: '%s'", result->ok ? "ros2_control: successfully switched controllers" : "ros2_control: failed to switch controllers");
          response->message = result->ok ? "ros2_control: successfully switched controllers" : "ros2_control: failed to switch controllers";
          response->success = result->ok;
          break;
      }
    }

    RCLCPP_INFO(logger, "Connection finished");
  }

  double ManagerBase::scale_stiffness(double value, double min, double max){
    double clamped_value = std::min(std::max(value, 0.0), 1.0);
    
    if(clamped_value != value){
      RCLCPP_WARN(node_->get_logger(), "Received stiffness value %f is not in the valid range [0.0, 1.0], clamping to %f", value, clamped_value);
    }

    return clamped_value * (max - min) + min;
  }

  void ManagerBase::load_compliant_controller_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    switch_controller(compliant_controllers_, default_controllers_, request, response);
  }

  void ManagerBase::load_default_controller_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    switch_controller(default_controllers_, compliant_controllers_, request, response);

  }

} // namespace rcm_manager