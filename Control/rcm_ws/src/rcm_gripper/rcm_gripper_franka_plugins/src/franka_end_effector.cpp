#include <chrono>
#include <functional>
#include <string>

#include "rclcpp/logging.hpp"
#include "rcm_gripper_franka_plugins/franka_end_effector.hpp"

using namespace std::chrono_literals;

namespace rcm_gripper_franka_plugins
{
  void FrankaEndEffector::init(std::shared_ptr<rclcpp::Node> node){

    // Setup
    init_base(node);
    this->max_width_ = 0;

    auto logger = node_->get_logger();

    init_cli();

    // Homing procedure
    if(!homing()) {
      RCLCPP_WARN_STREAM(logger, "Homing failed");
    }

    RCLCPP_INFO_STREAM(logger, "Franka Hand Plugin Initialized");
  }


  bool FrankaEndEffector::move(double width){
    auto logger = node_->get_logger();

    if(!stop()){
      RCLCPP_ERROR_STREAM(logger, "Franka stop service call failed");
      return false;
    }

    auto move_msg = Move::Goal();
    move_msg.width = width;
    move_msg.speed = speed_;

    auto goal_handle_future = move_client_->async_send_goal(move_msg, move_options_);

    bool success = false;

    // Wait for goal to be accepted
    for(int i=0; i<10; i++){
      RCLCPP_INFO(logger, "Waiting for response from /franka_gripper/move...");

      if(goal_handle_future.wait_for(1s) == std::future_status::timeout) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(logger, "Interrupted while waiting for /franka_gripper/move. Exiting.");
            return success;
        }
        else if (i>8){
            RCLCPP_ERROR(logger, "Timed out while waiting for /franka_gripper/move. Exiting.");
            return success;
        }
      }
      else {
        break;
      }
    }

    auto result_future = move_client_->async_get_result(goal_handle_future.get());
  
    // Wait for result
    for(int i=0; i<10; i++){
      if(result_future.wait_for(1s) == std::future_status::timeout) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(logger, "Interrupted while waiting to get a result from /franka_gripper/move. Exiting.");
            return success;
        }
        else if (i>8){
            RCLCPP_ERROR(logger, "Timed out while waiting to get a result from /franka_gripper/move. Exiting.");
            return success;
        }
      }
      else {
        break;
      }
    }

    auto result = result_future.get();

    if(result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      success = result.result->success;

      if(result.result->success) {
        RCLCPP_INFO_STREAM(logger, "Moving succesful!");
      }
      else {
        RCLCPP_ERROR_STREAM(logger, "Moving failed: " << result.result->error);
      }
    }

    return success;
  }


  bool FrankaEndEffector::grasp(double force){
    auto logger = node_->get_logger();

    if(!stop()){
      RCLCPP_ERROR_STREAM(logger, "Franka stop service call failed");
      return false;
    }

    bool success = false;

    // Wait action server to be ready
    for(int i=0; i<10; i++){
      RCLCPP_INFO(logger, "Waiting for /franka_gripper/grasp to become available...");

      if(!grasp_client_->wait_for_action_server(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(logger, "Interrupted while waiting for /franka_gripper/grasp. Exiting.");
            return success;
        }
        else if (i>8){
            RCLCPP_ERROR(logger, "Timed out while waiting for /franka_gripper/grasp. Exiting.");
            return success;
        }
      }
      else {
        break;
      }
    }

    auto grasp_msg = Grasp::Goal();
    grasp_msg.width = 0;
    grasp_msg.epsilon.outer = max_width_; // Grasp regardless of width
    grasp_msg.speed = speed_;
    grasp_msg.force = force;

    auto goal_handle_future = grasp_client_->async_send_goal(grasp_msg, grasp_options_);

    // Wait for goal to be accepted
    for(int i=0; i<10; i++){
      RCLCPP_INFO(logger, "Waiting for response from /franka_gripper/grasp...");

      if(goal_handle_future.wait_for(1s) == std::future_status::timeout) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(logger, "Interrupted while waiting for /franka_gripper/grasp. Exiting.");
            return success;
        }
        else if (i>8){
            RCLCPP_ERROR(logger, "Timed out while waiting for /franka_gripper/grasp. Exiting.");
            return success;
        }
      }
      else {
        break;
      }
    }

    auto result_future = grasp_client_->async_get_result(goal_handle_future.get());
  
    // Wait for result
    for(int i=0; i<10; i++){
      if(result_future.wait_for(1s) == std::future_status::timeout) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(logger, "Interrupted while waiting to get a result from /franka_gripper/grasp. Exiting.");
            return success;
        }
        else if (i>8){
            RCLCPP_ERROR(logger, "Timed out while waiting to get a result from /franka_gripper/grasp. Exiting.");
            return success;
        }
      }
      else {
        break;
      }
    }

    auto result = result_future.get();

    if(result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      success = result.result->success;

      if(result.result->success) {
        RCLCPP_INFO_STREAM(logger, "Moving succesful!");
      }
      else {
        RCLCPP_ERROR_STREAM(logger, "Moving failed: " << result.result->error);
      }
    }

    return success;
  }


  bool FrankaEndEffector::release(){
    auto logger = node_->get_logger();

    if(!stop()){
      RCLCPP_ERROR_STREAM(logger, "Franka stop service call failed");
      return false;
    }

    bool success = false;

    // Wait action server to be ready
    for(int i=0; i<10; i++){
      RCLCPP_INFO(logger, "Waiting for /franka_gripper/move to become available...");

      if(!move_client_->wait_for_action_server(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(logger, "Interrupted while waiting for /franka_gripper/move. Exiting.");
            return success;
        }
        else if (i>8){
            RCLCPP_ERROR(logger, "Timed out while waiting for /franka_gripper/move. Exiting.");
            return success;
        }
      }
      else {
        break;
      }
    }

    auto move_msg = Move::Goal();
    move_msg.width = 0.98*max_width_; // Leaving a bit of margin to avoid errors
    move_msg.speed = speed_;

    auto goal_handle_future = move_client_->async_send_goal(move_msg, move_options_);

    // Wait for goal to be accepted
    for(int i=0; i<10; i++){
      RCLCPP_INFO(logger, "Waiting for response from /franka_gripper/move...");

      if(goal_handle_future.wait_for(1s) == std::future_status::timeout) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(logger, "Interrupted while waiting for /franka_gripper/move. Exiting.");
            return success;
        }
        else if (i>8){
            RCLCPP_ERROR(logger, "Timed out while waiting for /franka_gripper/move. Exiting.");
            return success;
        }
      }
      else {
        break;
      }
    }

    auto result_future = move_client_->async_get_result(goal_handle_future.get());
  
    // Wait for result
    for(int i=0; i<10; i++){
      if(result_future.wait_for(1s) == std::future_status::timeout) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(logger, "Interrupted while waiting to get a result from /franka_gripper/move. Exiting.");
            return success;
        }
        else if (i>8){
            RCLCPP_ERROR(logger, "Timed out while waiting to get a result from /franka_gripper/move. Exiting.");
            return success;
        }
      }
      else {
        break;
      }
    }

    auto result = result_future.get();

    if(result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      success = result.result->success;

      if(result.result->success) {
        RCLCPP_INFO_STREAM(logger, "Moving succesful!");
      }
      else {
        RCLCPP_ERROR_STREAM(logger, "Moving failed: " << result.result->error);
      }
    }

    return success;
  }


  bool FrankaEndEffector::homing(){
    auto logger = node_->get_logger();

    bool success = false;

    // Wait action server to be ready
    for(int i=0; i<10; i++){
      RCLCPP_INFO(logger, "Waiting for /franka_gripper/homing to become available...");

      if(!homing_client_->wait_for_action_server(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(logger, "Interrupted while waiting for /franka_gripper/homing. Exiting.");
            return success;
        }
        else if (i>8){
            RCLCPP_ERROR(logger, "Timed out while waiting for /franka_gripper/homing. Exiting.");
            return success;
        }
      }
      else {
        break;
      }
    }

    auto homing_msg = Homing::Goal();
    auto goal_handle_future = homing_client_->async_send_goal(homing_msg, this->homing_options_);

    // Wait for goal to be accepted
    for(int i=0; i<10; i++){
      RCLCPP_INFO(logger, "Waiting for response from /franka_gripper/homing...");

      if(goal_handle_future.wait_for(1s) == std::future_status::timeout) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(logger, "Interrupted while waiting for /franka_gripper/homing. Exiting.");
            return success;
        }
        else if (i>8){
            RCLCPP_ERROR(logger, "Timed out while waiting for /franka_gripper/homing. Exiting.");
            return success;
        }
      }
      else {
        break;
      }
    }

    auto result_future = homing_client_->async_get_result(goal_handle_future.get());
  
    // Wait for result
    for(int i=0; i<10; i++){
      if(result_future.wait_for(1s) == std::future_status::timeout) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(logger, "Interrupted while waiting to get a result from /franka_gripper/homing. Exiting.");
            return success;
        }
        else if (i>8){
            RCLCPP_ERROR(logger, "Timed out while waiting to get a result from /franka_gripper/homing. Exiting.");
            return success;
        }
      }
      else {
        break;
      }
    }

    auto result = result_future.get();

    if(result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      success = result.result->success;

      if(result.result->success) {
        RCLCPP_INFO_STREAM(logger, "Homing succesful!");
      }
      else {
        RCLCPP_ERROR_STREAM(logger, "Homing failed: " << result.result->error);
      }
    }
    return success;
  }

  void FrankaEndEffector::end_effector_goal_execute(const std::shared_ptr<GoalHandleControlEndEffector> goal_handle) {
    auto logger = node_->get_logger();
    
    // Action setup
    RCLCPP_INFO_STREAM(logger, "Processing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ControlEndEffector::Feedback>();
    auto result = std::make_shared<ControlEndEffector::Result>();
    
    std::string msg = "Unknown";
    feedback->state = msg;
    result->success = false;
    result->state = msg;

    if(goal->io_name.size() != goal->io_state.size()) {
      msg = "io_name and io_state must be the same size";
      send_feedback(goal_handle, msg, "ERROR");

      result->success = false;
      result->state = "Did not start execution, process exited with an error";
      goal_handle->succeed(result);
      return;
    }

    bool cmd_received = false;
    bool releasing = false;
    bool grasping = false;
    bool moving = false;

    for(size_t i=0; i < goal->io_name.size(); i++) {
      if(goal->io_name[i] == "gripper") {
        cmd_received = true;
        releasing = goal->io_state[i] == 0;
        grasping = goal->io_state[i] == 1;
        moving = goal->io_state[i] == 2;

        msg = "Accepted command for IO: \"gripper\"";
        send_feedback(goal_handle, msg, "INFO");
      }
      else {
        msg = "Unsupported IO: \"" + goal->io_name[i] + "\", ignoring";
        send_feedback(goal_handle, msg, "WARN");
      }
    }

    if(!cmd_received){
      msg = "Did not receive a valid IO name, options: [\"gripper\"]";
      send_feedback(goal_handle, msg, "ERROR");
      result->success = false;
    }
    else if(releasing){
      msg = "Releasing...";
      send_feedback(goal_handle, msg, "INFO");
      result->success = release();
    }
    else if(grasping){
      msg = "Grasping with <force>...";
      send_feedback(goal_handle, msg, "INFO");
      result->success = grasp(goal->force);
    }
    else if(moving){
      msg = "Moving to <width>...";
      send_feedback(goal_handle, msg, "INFO");
      result->success = move(goal->width);
    }
    else{
      msg = "Did not receive a valid IO state, options: [0, 1, 2] (=[release, grasp, move])";
      send_feedback(goal_handle, msg, "ERROR");
      result->success = false;
    }

    msg = "IO processed";
    send_feedback(goal_handle, msg, "INFO");

    if(result->success){
      msg = "Action successfully completed";
    }
    else{
      msg = "Error(s) during execution";
    }

    result->state = msg;
    goal_handle->succeed(result);
  }

  void FrankaEndEffector::send_feedback(const std::shared_ptr<GoalHandleControlEndEffector> goal_handle,
    std::string msg, std::string msg_style){
    auto logger = node_->get_logger();
    auto feedback = std::make_shared<ControlEndEffector::Feedback>();

    if(msg_style == "INFO"){
      RCLCPP_INFO_STREAM(logger, msg);
      feedback->state = msg;
      goal_handle->publish_feedback(feedback);
    }
    else if(msg_style == "WARN"){
      RCLCPP_WARN_STREAM(logger, msg);
      feedback->state = "[" + msg_style + "] " + msg;
      goal_handle->publish_feedback(feedback);
    }
    else if(msg_style == "ERROR"){
      RCLCPP_ERROR_STREAM(logger, msg);
      feedback->state = "[" + msg_style + "] " + msg;
      goal_handle->publish_feedback(feedback);
    }
    else{
      RCLCPP_WARN_STREAM(logger, "[" + msg_style + "] " << msg);
      feedback->state = "[" + msg_style + "] " + msg;
      goal_handle->publish_feedback(feedback);
    }
  }

  bool FrankaEndEffector::stop(){
    std::string msg = "Unknown";
    auto logger = node_->get_logger();

    bool success = false;

    int n = 0;

    while (!stop_client_->wait_for_service(1s) && n < 10) {
      ++n;
      RCLCPP_WARN(logger, "Service /franka_gripper/stop not available, trying again...");
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(logger, "Interrupted while waiting for /franka_gripper/stop. Exiting.");
        return success;
      }
      else if(n>9) {
        RCLCPP_ERROR(logger, "Timeout while waiting for /franka_gripper/stop. Exiting.");
        return success;
      }
    }

    std::future_status status;
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result_future = stop_client_->async_send_request(request);

    switch (status = result_future.wait_for(5s); status){
      case std::future_status::deferred:
        msg = "Did not receive a response from /franka_gripper/stop";
        RCLCPP_INFO_STREAM(node_->get_logger(), msg);
        success = false;
        break;

      case std::future_status::timeout:
        msg = "Did not receive a response from /franka_gripper/stop";
        RCLCPP_INFO_STREAM(node_->get_logger(), msg);
        success = false;
        break;

      case std::future_status::ready:
        auto result = result_future.get();
        success = result->success;
        if(success){
          msg = "Stop call successful, message:";
          RCLCPP_INFO_STREAM(node_->get_logger(), msg << result->message);
        }
        else{
          msg = "Stop call failed, message:";
          RCLCPP_INFO_STREAM(node_->get_logger(), msg << result->message);
        }
        break;
    }
  
    return success;
  }

  void FrankaEndEffector::init_cli(){
    using namespace std::placeholders;

    // Stop service
    this->stop_client_ =
    this->node_->create_client<std_srvs::srv::Trigger>("/franka_gripper/stop");

    // Homing action
    this->homing_client_ = rclcpp_action::create_client<Homing>(
      this->node_,
      "/franka_gripper/homing");
    
    this->homing_options_ = rclcpp_action::Client<Homing>::SendGoalOptions();
    
    homing_options_.goal_response_callback =
      std::bind(&FrankaEndEffector::homing_response_callback, this, _1);
    homing_options_.feedback_callback =
      std::bind(&FrankaEndEffector::homing_feedback_callback, this, _1, _2);
    homing_options_.result_callback =
      std::bind(&FrankaEndEffector::homing_result_callback, this, _1);

    // Grasp action
    this->grasp_client_ = rclcpp_action::create_client<Grasp>(
      this->node_,
      "/franka_gripper/grasp");
    
    this->grasp_options_ = rclcpp_action::Client<Grasp>::SendGoalOptions();
    
    grasp_options_.goal_response_callback =
      std::bind(&FrankaEndEffector::grasp_response_callback, this, _1);
    grasp_options_.feedback_callback =
      std::bind(&FrankaEndEffector::grasp_feedback_callback, this, _1, _2);
    grasp_options_.result_callback =
      std::bind(&FrankaEndEffector::grasp_result_callback, this, _1);

    // Move action
    this->move_client_ = rclcpp_action::create_client<Move>(
      this->node_,
      "/franka_gripper/move");
    
    this->move_options_ = rclcpp_action::Client<Move>::SendGoalOptions();
    
    move_options_.goal_response_callback =
      std::bind(&FrankaEndEffector::move_response_callback, this, _1);
    move_options_.feedback_callback =
      std::bind(&FrankaEndEffector::move_feedback_callback, this, _1, _2);
    move_options_.result_callback =
      std::bind(&FrankaEndEffector::move_result_callback, this, _1);
  }

  void FrankaEndEffector::homing_response_callback(const HomingGoalHandle::SharedPtr & goal_handle){
    auto logger = node_->get_logger();
    if (!goal_handle) {
      RCLCPP_ERROR_STREAM(logger, "Homing goal was rejected by Franka Hand");
    } else {
      RCLCPP_INFO_STREAM(logger, "Homing goal accepted by Franka Hand, waiting for result");
    }
  }
  
  void FrankaEndEffector::homing_feedback_callback(HomingGoalHandle::SharedPtr,
    const std::shared_ptr<const Homing::Feedback> feedback){
    max_width_ = (max_width_ < feedback->current_width) ? feedback->current_width : max_width_;
  }
  
  void FrankaEndEffector::homing_result_callback(const HomingGoalHandle::WrappedResult & result){
    auto logger = node_->get_logger();

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR_STREAM(logger, "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR_STREAM(logger, "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR_STREAM(logger, "Unknown result code");
        break;
    }

  }
   
  void FrankaEndEffector::grasp_response_callback(const GraspGoalHandle::SharedPtr & goal_handle){
    auto logger = node_->get_logger();
    if (!goal_handle) {
      RCLCPP_ERROR_STREAM(logger, "Grasp goal was rejected by Franka Hand");
    } else {
      RCLCPP_INFO_STREAM(logger, "Grasp goal accepted by Franka Hand, waiting for result");
    }
  }
      
  void FrankaEndEffector::grasp_feedback_callback(GraspGoalHandle::SharedPtr,
    const std::shared_ptr<const Grasp::Feedback> feedback){
    (void)feedback;
  }

  void FrankaEndEffector::grasp_result_callback(const GraspGoalHandle::WrappedResult & result){
    auto logger = node_->get_logger();

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR_STREAM(logger, "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR_STREAM(logger, "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR_STREAM(logger, "Unknown result code");
        break;
    }

  }

  void FrankaEndEffector::move_response_callback(const MoveGoalHandle::SharedPtr & goal_handle){
    auto logger = node_->get_logger();
    if (!goal_handle) {
      RCLCPP_ERROR_STREAM(logger, "Move goal was rejected by Franka Hand");
    } else {
      RCLCPP_INFO_STREAM(logger, "Move goal accepted by Franka Hand, waiting for result");
    }
  }

  void FrankaEndEffector::move_feedback_callback(MoveGoalHandle::SharedPtr,
    const std::shared_ptr<const Move::Feedback> feedback){
    (void)feedback;
  }

  void FrankaEndEffector::move_result_callback(const MoveGoalHandle::WrappedResult & result){
    auto logger = node_->get_logger();

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR_STREAM(logger, "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR_STREAM(logger, "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR_STREAM(logger, "Unknown result code");
        break;
    }

  }

} //namespace rcm_gripper_franka_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rcm_gripper_franka_plugins::FrankaEndEffector, rcm_gripper::GenericEndEffector)