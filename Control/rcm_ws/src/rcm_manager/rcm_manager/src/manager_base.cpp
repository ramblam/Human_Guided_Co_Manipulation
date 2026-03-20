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

    std::vector<std::string> default_teleop_controllers = {};
    node_->declare_parameter("default_teleop_controllers", default_teleop_controllers);
    node_->get_parameter("default_teleop_controllers", default_teleop_controllers_);
    RCLCPP_INFO(logger, "Loaded param: default_teleop_controllers:=%s", str_vec_to_str(default_teleop_controllers_).c_str());

    std::vector<std::string> compliant_controllers = {"fr3_arm_compliant_controller"};
    node_->declare_parameter("compliant_controllers", compliant_controllers);
    node_->get_parameter("compliant_controllers", compliant_controllers_);
    RCLCPP_INFO(logger, "Loaded param: compliant_controllers:=%s", str_vec_to_str(compliant_controllers_).c_str());

    std::vector<std::string> compliant_teleop_controllers = {};
    node_->declare_parameter("compliant_teleop_controllers", compliant_teleop_controllers);
    node_->get_parameter("compliant_teleop_controllers", compliant_teleop_controllers_);
    RCLCPP_INFO(logger, "Loaded param: compliant_teleop_controllers:=%s", str_vec_to_str(compliant_teleop_controllers_).c_str());

    std::vector<std::string> managed_controllers = {};
    managed_controllers.insert(managed_controllers.end(), default_controllers_.begin(), default_controllers_.end());
    managed_controllers.insert(managed_controllers.end(), default_teleop_controllers_.begin(), default_teleop_controllers_.end());
    managed_controllers.insert(managed_controllers.end(), compliant_controllers_.begin(), compliant_controllers_.end());
    managed_controllers.insert(managed_controllers.end(), compliant_teleop_controllers_.begin(), compliant_teleop_controllers_.end());
    node_->declare_parameter("managed__controllers", managed_controllers);
    node_->get_parameter("managed__controllers", managed_controllers_);
    RCLCPP_INFO(logger, "Loaded param: managed_controllers:=%s", str_vec_to_str(managed_controllers_).c_str());

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
    load_compliant_controller_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "rcm_manager/load_compliant_controller", 
      std::bind(&ManagerBase::load_compliant_controller_cb, this, _1, _2, _3));
    
    load_default_controller_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "rcm_manager/load_default_controller", 
      std::bind(&ManagerBase::load_default_controller_cb, this, _1, _2, _3));

    set_speed_service_ = node_->create_service<rcm_msgs::srv::SetSpeed>(
      "rcm_manager/set_speed", 
      std::bind(&ManagerBase::set_speed_cb, this, _1, _2, _3));
    
    set_stiffness_service_ = node_->create_service<rcm_msgs::srv::SetStiffness>(
      "rcm_manager/set_stiffness", 
      std::bind(&ManagerBase::set_stiffness_cb, this, _1, _2, _3));

    switch_controller_client_ = node_->create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller");
    
    set_speed_client_ = node_->create_client<rcm_msgs::srv::SetSpeed>(
      "/rcm_moveit/set_speed");

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

  void ManagerBase::async_switch_controller(const std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service,
    const std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::vector<std::string> activate_controllers){

    auto logger = node_->get_logger();

    // If there's nothing to activate, no need to make the service call to ros2_control
    if(activate_controllers.size() == 0){
      RCLCPP_ERROR(logger, "List of controllers to activate is empty, aborted");
      std_srvs::srv::Trigger::Response response;
      response.message = "no controllers in activate list, call aborted";
      response.success = false;
      service->send_response(*request_header, response);
      return;
    }

    std::vector<std::string> deactivate_controllers = get_controllers_to_deactivate(activate_controllers);

    // Lambda to asynchronously perform and respond to the service call
    auto async_cb = [service, request_header, request](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) {
      (void)request;
      std_srvs::srv::Trigger::Response response;
      response.message = future.get()->ok ? "ros2_control: successfully switched controllers" : "ros2_control: failed to switch controllers";
      response.success = future.get()->ok;
      service->send_response(*request_header, response);
    };

    auto request_inner = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request_inner->activate_controllers = activate_controllers;
    request_inner->deactivate_controllers = deactivate_controllers;
    request_inner->strictness = 1;

    RCLCPP_INFO(logger, "Sending asynchronous request to switch controllers");
    switch_controller_client_->async_send_request(request_inner, async_cb);
    RCLCPP_INFO(logger, "Releasing executor");
  }

  double ManagerBase::scale_stiffness(double value, double min, double max){
    double clamped_value = std::min(std::max(value, 0.0), 1.0);
    
    if(clamped_value != value){
      RCLCPP_WARN(node_->get_logger(), "Received normalized stiffness value %f is not in the valid range [0.0, 1.0], clamping to %f", value, clamped_value);
    }

    double adjusted_value = clamped_value * (max - min) + min;

    RCLCPP_INFO(node_->get_logger(), "Converted normalized stiffness value %f to %f (min: %f, max: %f)", value, adjusted_value, min, max);

    return adjusted_value;
  }

  std::vector<std::string> ManagerBase::get_controllers_to_deactivate(std::vector<std::string> activate_controllers){
    std::vector<std::string> deactivate_controllers = {};

    // ros2_control will not complain about deactivating controllers that aren't active,
    // so we can just deactivate every managed controller that isn't being activated
    for(const auto &managed : managed_controllers_){
      bool is_active = false;

      for(const auto &active : activate_controllers){
        if(managed == active){
          is_active = true;
          break;
        }
      }

      if(!is_active){
        deactivate_controllers.push_back(managed);
      }
    }

    return deactivate_controllers;
  }

  void ManagerBase::load_compliant_controller_cb(const std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service,
    const std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<std_srvs::srv::Trigger::Request> request) {

    async_switch_controller(service, request_header, request, compliant_controllers_);
  }

  void ManagerBase::load_compliant_teleop_controller_cb(const std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service,
    const std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<std_srvs::srv::Trigger::Request> request) {

    async_switch_controller(service, request_header, request, compliant_teleop_controllers_);
  }

  void ManagerBase::load_default_controller_cb(const std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service,
    const std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<std_srvs::srv::Trigger::Request> request) {
    
    async_switch_controller(service, request_header, request, default_controllers_);
  }

  void ManagerBase::load_default_teleop_controller_cb(const std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service,
    const std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<std_srvs::srv::Trigger::Request> request) {
    
    async_switch_controller(service, request_header, request, default_teleop_controllers_);
  }

  void ManagerBase::set_speed_cb(const std::shared_ptr<rclcpp::Service<rcm_msgs::srv::SetSpeed>> service,
    const std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<rcm_msgs::srv::SetSpeed::Request> request){

    auto logger = node_->get_logger();

    // Lambda to asynchronously perform and respond to the service call
    auto async_cb = [service, request_header, request](rclcpp::Client<rcm_msgs::srv::SetSpeed>::SharedFuture future) {
      (void)request;
      rcm_msgs::srv::SetSpeed::Response response;
      response.success = future.get()->success;
      service->send_response(*request_header, response);
    };

    auto request_inner = std::make_shared<rcm_msgs::srv::SetSpeed::Request>();
    request_inner->speed = request->speed;

    RCLCPP_INFO(logger, "Sending asynchronous request to set speed");
    set_speed_client_->async_send_request(request_inner, async_cb);
    RCLCPP_INFO(logger, "Releasing executor");
  }

} // namespace rcm_manager