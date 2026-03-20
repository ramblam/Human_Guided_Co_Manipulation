#include "rcm_clients/pick_and_place_client.hpp"

namespace rcm_clients {
  PickAndPlaceClient::PickAndPlaceClient(std::shared_ptr<rclcpp::Node>& node){
    using namespace std::placeholders;

    action_name_ = "/rcm_pick_and_place/pick_and_place";
    node_ = node;
    pick_and_place_client_ = rclcpp_action::create_client<PickAndPlace>(
        node_,
        action_name_);
      
    pick_and_place_options_ = rclcpp_action::Client<PickAndPlace>::SendGoalOptions();
    pick_and_place_options_.goal_response_callback =
      std::bind(&PickAndPlaceClient::pick_and_place_response_callback, this, _1);
    pick_and_place_options_.feedback_callback =
      std::bind(&PickAndPlaceClient::pick_and_place_feedback_callback, this, _1, _2);
    pick_and_place_options_.result_callback =
      std::bind(&PickAndPlaceClient::pick_and_place_result_callback, this, _1);
  }
    
 bool PickAndPlaceClient::pick_and_place(geometry_msgs::msg::Pose pick_pose, 
    geometry_msgs::msg::Pose place_pose, double distance, double force, bool advanced_mode){
    using namespace std::chrono_literals;
    
    auto logger = node_->get_logger();
    bool success = false;
    
    auto pick_and_place_msg = PickAndPlace::Goal();
    pick_and_place_msg.pick_pose = pick_pose;
    pick_and_place_msg.place_pose = place_pose;
    pick_and_place_msg.distance = distance;
    pick_and_place_msg.force = force;
    pick_and_place_msg.advanced_mode = advanced_mode;

    success = wait_for_result(pick_and_place_msg);

    return success;
  }

  bool PickAndPlaceClient::pick(geometry_msgs::msg::Pose pick_pose, double distance, double force, bool advanced_mode){
    using namespace std::chrono_literals;
    
    geometry_msgs::msg::Pose place_pose;

    auto logger = node_->get_logger();
    bool success = false;
    auto pick_msg = PickAndPlace::Goal();
    pick_msg.pick_pose = pick_pose;
    pick_msg.place_pose = place_pose;
    pick_msg.place = false;
    pick_msg.distance = distance;
    pick_msg.force = force;
    pick_msg.advanced_mode = advanced_mode;

    success = wait_for_result(pick_msg);

    return success;
  }

  bool PickAndPlaceClient::place(geometry_msgs::msg::Pose place_pose, double distance, double force, bool advanced_mode){
    using namespace std::chrono_literals;
    
    geometry_msgs::msg::Pose pick_pose;

    auto logger = node_->get_logger();
    bool success = false;
    auto place_msg = PickAndPlace::Goal();
    place_msg.pick_pose = pick_pose;
    place_msg.place_pose = place_pose;
    place_msg.pick = false;
    place_msg.distance = distance;
    place_msg.force = force;
    place_msg.advanced_mode = advanced_mode;

    success = wait_for_result(place_msg);

    return success;
  }

  bool PickAndPlaceClient::wait_for_result(PickAndPlace::Goal goal_msg){
    using namespace std::chrono_literals;
    
    auto logger = node_->get_logger();
    bool success = false;

    // Wait for action server to be ready
    for(int i=0; i<10; i++){
      RCLCPP_INFO(logger, "Waiting for %s to be available...", action_name_.c_str());

      if(!pick_and_place_client_->wait_for_action_server(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(logger, "Interrupted while waiting for %s. Exiting.", action_name_.c_str());
            return success;
        }
        else if (i>8){
            RCLCPP_ERROR(logger, "Timed out while waiting for %s. Exiting.", action_name_.c_str());
            return success;
        }
      }
      else {
        break;
      }
    }

    auto goal_handle_future = pick_and_place_client_->async_send_goal(goal_msg, pick_and_place_options_);
    
    // Wait for goal to be accepted
    for(int i=0; i<10; i++){
      RCLCPP_INFO(logger, "Waiting for response from %s...", action_name_.c_str());

      if(goal_handle_future.wait_for(1s) == std::future_status::timeout) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(logger, "Interrupted while waiting for %s. Exiting.", action_name_.c_str());
            return success;
        }
        else if (i>8){
            RCLCPP_ERROR(logger, "Timed out while waiting for %s. Exiting.", action_name_.c_str());
            return success;
        }
      }
      else {
        break;
      }
    }

    auto result_future = pick_and_place_client_->async_get_result(goal_handle_future.get());
  
    // Wait for result
    result_future.wait();
    for(int i=0; i<10; i++){
      if(result_future.wait_for(1s) == std::future_status::timeout) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(logger, "Interrupted while waiting to get a result from %s. Exiting.", action_name_.c_str());
            return success;
        }
        else if (i>8){
            RCLCPP_ERROR(logger, "Timed out while waiting to get a result from %s. Exiting.", action_name_.c_str());
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
    }

    return success;
  }

  void PickAndPlaceClient::pick_and_place_response_callback(const GoalHandlePickAndPlace::SharedPtr & goal_handle){
    auto logger = node_->get_logger();
    if (!goal_handle) {
      RCLCPP_ERROR_STREAM(logger, "Goal was rejected by " << action_name_);
    } else {
      RCLCPP_INFO_STREAM(logger, "Goal was accepted by " << action_name_);
    }
  }

  void PickAndPlaceClient::pick_and_place_feedback_callback(GoalHandlePickAndPlace::SharedPtr,
    const std::shared_ptr<const PickAndPlace::Feedback> feedback){
    RCLCPP_INFO(node_->get_logger(), "State: %s", feedback->state.c_str());
  }

  void PickAndPlaceClient::pick_and_place_result_callback(const GoalHandlePickAndPlace::WrappedResult & result){
    auto logger = node_->get_logger();

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(logger, "Goal completed: %s", result.result->success ? "SUCCESS" : "FAILED");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(logger, "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(logger, "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(logger, "Unknown result code");
        break;
    }
  }
} // namespace rcm_clients
