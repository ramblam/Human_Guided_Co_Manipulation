#include <rclcpp/logging.hpp>
#include "rcm_manager_cic_plugins/cartesian_impedance_controller_manager.hpp"

namespace rcm_manager_cic_plugins
{
  void CartesianImpedanceControllerManager::init(std::shared_ptr<rclcpp::Node> node){
    init_base(node);
    
    auto logger = node_->get_logger();

    set_cic_parameters_client_ = node_->create_client<rcl_interfaces::srv::SetParametersAtomically>(
      stiffness_param_service_);

    RCLCPP_INFO(logger, "CartesianImpedanceControllerManager plugin initialized");
  }

  void CartesianImpedanceControllerManager::set_stiffness_cb(const std::shared_ptr<rclcpp::Service<rcm_msgs::srv::SetStiffness>> service,
    const std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<rcm_msgs::srv::SetStiffness::Request> request) {

    auto logger = node_->get_logger();

    // Lambda to asynchronously perform and respons to the service call
    auto async_cb = [service, request_header, request](rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedFuture future) {
      (void)request;
      rcm_msgs::srv::SetStiffness::Response response;
      response.message = future.get()->result.successful ? "success" : future.get()->result.reason;
      response.success = future.get()->result.successful;
      service->send_response(*request_header, response);
    };

    auto request_inner = std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();
    
    auto stiffness_force_x = rcl_interfaces::msg::Parameter();
    auto stiffness_force_y = rcl_interfaces::msg::Parameter();
    auto stiffness_force_z = rcl_interfaces::msg::Parameter();
    auto stiffness_torque_x = rcl_interfaces::msg::Parameter();
    auto stiffness_torque_y = rcl_interfaces::msg::Parameter();
    auto stiffness_torque_z = rcl_interfaces::msg::Parameter();
    
    stiffness_force_x.name = "stiffness.force.x";
    stiffness_force_x.value.type = 3;
    stiffness_force_x.value.double_value = scale_stiffness(request->stiffness.force.x, trans_stiffness_min_, trans_stiffness_max_);

    stiffness_force_y.name = "stiffness.force.y";
    stiffness_force_y.value.type = 3;
    stiffness_force_y.value.double_value = scale_stiffness(request->stiffness.force.y, trans_stiffness_min_, trans_stiffness_max_);

    stiffness_force_z.name = "stiffness.force.z";
    stiffness_force_z.value.type = 3;
    stiffness_force_z.value.double_value = scale_stiffness(request->stiffness.force.z, trans_stiffness_min_, trans_stiffness_max_);

    stiffness_torque_x.name = "stiffness.torque.x";
    stiffness_torque_x.value.type = 3;
    stiffness_torque_x.value.double_value = scale_stiffness(request->stiffness.torque.x, rot_stiffness_min_, rot_stiffness_max_);

    stiffness_torque_y.name = "stiffness.torque.y";
    stiffness_torque_y.value.type = 3;
    stiffness_torque_y.value.double_value = scale_stiffness(request->stiffness.torque.y, rot_stiffness_min_, rot_stiffness_max_);

    stiffness_torque_z.name = "stiffness.torque.z";
    stiffness_torque_z.value.type = 3;
    stiffness_torque_z.value.double_value = scale_stiffness(request->stiffness.torque.z, rot_stiffness_min_, rot_stiffness_max_);
    
    request_inner->parameters = {
      stiffness_force_x,
      stiffness_force_y,
      stiffness_force_z,
      stiffness_torque_x,
      stiffness_torque_y,
      stiffness_torque_z,
    };

    RCLCPP_INFO(logger, "Sending asynchronous request to set stiffness parameters");
    set_cic_parameters_client_->async_send_request(request_inner, async_cb);
    RCLCPP_INFO(logger, "Releasing executor");
  }

}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rcm_manager_cic_plugins::CartesianImpedanceControllerManager, rcm_manager::ManagerBase)