#include "co_manipulation_pkg/own_controller.hpp"
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

OwnController::OwnController(std::shared_ptr<rclcpp::Node>& node)
: control_end_effector_client_(node), get_pose_client_(node), cartesian_movement_client_(node), joint_movement_client_(node), get_joints_client_(node), rcm_manager_client_(node), pose_movement_client_(node)
{
  node_ = node;
  io_name_ = {"gripper"};
  io_state_ = {0};
  width_ = 0.0;
  force_ = 0.5;
  angle_ = 0.0;

  stiffness_step_size_ = 0.1;
  default_stiffness_ = 0.5;
  current_speed_ = 0;

  home_joint_values_ = {0.00093,-0.78429,-0.00089,-2.35567,0.00286,1.57407,-0.72409};
  compliance_controller_activated_= true;
  rclcpp::Time now = node->get_clock()->now();
  ref_frame_publisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("/reference_pose", 10);
  r_wrist_pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/r_wrist_poses", 10, std::bind(&OwnController::r_wrist_pose_cb, this, std::placeholders::_1));

  remote_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "fr3_arm_compliant_controller");
  std::vector<double> ee_orientation = {0, 0, 0, 1};
  node->declare_parameter("default_ee_orientation", ee_orientation);
  node->get_parameter("default_ee_orientation", default_ee_orientation_);

  bool use_current_orientation = true;
  node->declare_parameter("use_current_orientation", true);
  node->get_parameter("use_current_orientation", use_current_orientation);

  if(use_current_orientation){
    geometry_msgs::msg::Pose current_pose;
    
    if(!get_pose_client_.update_pose(current_pose)){
      RCLCPP_WARN(node_->get_logger(), "Failed to update pose, can't get current orientation");
    }
    else{
      RCLCPP_INFO(node_->get_logger(), "End-effector orientation updated");
      default_ee_orientation_ = {current_pose.orientation.x,
                                  current_pose.orientation.y,
                                  current_pose.orientation.z,
                                  current_pose.orientation.w};
    }
  }
}

void OwnController::set_force(double force){
  force_= force;
}

bool OwnController::pick(const geometry_msgs::msg::Pose target_pose, double distance, bool advanced_mode){
  auto logger = node_->get_logger();

  RCLCPP_INFO(logger, "Picking sequence started");

  geometry_msgs::msg::Pose starting_pose;
  if(!get_pose_client_.update_pose(starting_pose)){
    RCLCPP_ERROR(logger, "Failed to update pose");
    return false;
  }

  if(!approach(target_pose, distance, advanced_mode)){
    RCLCPP_ERROR(logger, "Approach failed");
    return false;
  }

  io_state_[0] = 1;
  if(!control_end_effector_client_.control_end_effector(io_name_, io_state_, width_, force_, angle_)){
    RCLCPP_ERROR(logger, "Grasping failed");
    return false;
  }

  if(!retreat(starting_pose, target_pose, distance, advanced_mode)){
    RCLCPP_ERROR(logger, "Retreat failed");
    return false;
  }

  RCLCPP_INFO(logger, "Picking sequence completed");

  return true;
}

bool OwnController::place(const geometry_msgs::msg::Pose target_pose, double distance, bool advanced_mode){
  auto logger = node_->get_logger();

  RCLCPP_INFO(logger, "Placing sequence started");

  geometry_msgs::msg::Pose starting_pose;
  if(!get_pose_client_.update_pose(starting_pose)){
    RCLCPP_ERROR(logger, "Failed to update pose");
    return false;
  }

  if(!approach(target_pose, distance, advanced_mode)){
    RCLCPP_ERROR(logger, "Approach failed");
    return false;
  }

  io_state_[0] = 0;
  if(!control_end_effector_client_.control_end_effector(io_name_, io_state_, width_, force_, angle_)){
    RCLCPP_ERROR(logger, "Releasing failed");
    return false;
  }

  if(!retreat(starting_pose, target_pose, distance, advanced_mode)){
    RCLCPP_ERROR(logger, "Retreat failed");
    return false;
  }

  RCLCPP_INFO(logger, "Placing sequence completed");

  return true;
}

bool OwnController::control_end_effector(
    const std::vector<std::string>& io_name,
    const std::vector<long>& io_state,
    double width,
    double force,
    double angle)
{
    return control_end_effector_client_.control_end_effector(io_name, io_state, width, force, angle);
}

bool OwnController::open_gripper()
{
  // Releasing
  io_state_[0] = 0;
  io_name_ = {"gripper"};
  bool success = control_end_effector_client_.control_end_effector(io_name_, io_state_, width_, force_, angle_);
  if(!success){
      RCLCPP_ERROR(node_->get_logger(), "Failed to release");
      return false;
  }
  return true;
}

bool OwnController::close_gripper()
{
  // Releasing
  io_state_[0] = 1;
  io_name_ = {"gripper"};
  bool success = control_end_effector_client_.control_end_effector(io_name_, io_state_, width_, force_, angle_);
  if(!success){
      RCLCPP_ERROR(node_->get_logger(), "Failed to release");
      return false;
  }
  return true;
}

bool OwnController::update_current_pose(geometry_msgs::msg::Pose& pose)
{
    return get_pose_client_.update_pose(pose);
}

bool OwnController::move_to_pose(const geometry_msgs::msg::Pose& pose, double speed, double timeout)
{
    return pose_movement_client_.execute_pose_movement(pose, speed, timeout);
}

void OwnController::pick_ply(
    std::function<void(bool)> result_cb)
{
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    static_grasp_client_->async_send_request(
        request,
        [this, result_cb](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
        {
            auto response = future.get();
            result_cb(response->success);
        });
}

bool OwnController::move_to_joints(std::vector<double> joint_positions)
{
    return joint_movement_client_.execute_joint_movement(joint_positions);
}

bool OwnController::move_home()
{
    return joint_movement_client_.execute_joint_movement(home_joint_values_);
}

void OwnController::r_wrist_pose_cb(const geometry_msgs::msg::PoseStamped& r_wrist_pose)
{
  current_r_wrist_pose_ = r_wrist_pose.pose;
  if (wrist_following_servo_activated_) {
    follow_wrist_servo();
  } else if (wrist_following_ref_activated_) {
    follow_wrist_ref_frame();
  }
}

void OwnController::startOperationCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (operation_active_) {
      response->success = false;
      response->message = "Operation already running.";
      return;
    }

    operation_active_ = true;
    bool success = move_home();

    if (success) {
      response->success = true;
      response->message = "Operation started successfully.";
    } else {
      response->success = false;
      response->message = "Starting operation failed, couldn't move home.";
    }

    RCLCPP_INFO(this->node_->get_logger(), "Operation triggered.");
  }

void OwnController::follow_wrist_servo()
{
  auto logger = node_->get_logger();
  geometry_msgs::msg::PoseStamped target_pose_stamped;
  target_pose_stamped.header.stamp = node_->now();
  target_pose_stamped.header.frame_id = "fr3_link0";
  target_pose_stamped.pose.position.x = current_r_wrist_pose_.position.x;
  target_pose_stamped.pose.position.y = current_r_wrist_pose_.position.y;

  double big_ply_width = 0.75;
  double big_ply_length = 0.85;
  target_pose_stamped.pose.position.x -= big_ply_length;
  target_pose_stamped.pose.position.y -= big_ply_width*0.5;
  target_pose_stamped.pose.position.z = current_r_wrist_pose_.position.z;
  
  target_pose_stamped.pose.orientation.x = starting_pose_wrist_following_servo_.orientation.x;
  target_pose_stamped.pose.orientation.y = starting_pose_wrist_following_servo_.orientation.y;
  target_pose_stamped.pose.orientation.z = starting_pose_wrist_following_servo_.orientation.z;
  target_pose_stamped.pose.orientation.w = starting_pose_wrist_following_servo_.orientation.w;

  servo_pose_publisher_->publish(target_pose_stamped);
  prev_r_wrist_pose_ = current_r_wrist_pose_;
  wrist_following_round_servo_ +=1;
}

//This function is used for making the robot movement a bit slower and smoother for wrist tracking by lowering the pose filtering parameter:
void OwnController::start_safe_tracking() {

  RCLCPP_INFO(node_->get_logger(), "Requesting slow speed configuration for wrist tracking...");

  // 1. Send the parameter update request
  remote_client_->set_parameters_atomically(
  {
    rclcpp::Parameter("filtering.pose", 0.1),
    rclcpp::Parameter("stiffness.force.x", 50.0),
    rclcpp::Parameter("stiffness.force.y", 50.0),
    rclcpp::Parameter("stiffness.force.z", 50.0)
  },
  [this](std::shared_future<rcl_interfaces::msg::SetParametersResult> future) {
      try {
          // Get the single atomic result
          auto result = future.get();

          if (result.successful) {
              RCLCPP_INFO(node_->get_logger(), "Parameters applied atomically! Starting movement.");
              wrist_following_ref_activated_ = true;
          } else {
              // This triggers if parameters were out of bounds or rejected by the controller
              RCLCPP_ERROR(node_->get_logger(), "Atomic update rejected: %s", result.reason.c_str());
              wrist_following_ref_activated_ = false;
          }
      } catch (const std::exception & e) {
          // This triggers if the service call timed out or the remote node disappeared
          RCLCPP_ERROR(node_->get_logger(), "Service call failed: %s", e.what());
          wrist_following_ref_activated_ = false;
      }
  }
);
}

void OwnController::follow_wrist_ref_frame()
{
  geometry_msgs::msg::PoseStamped target_pose_stamped;
  target_pose_stamped.header.stamp = node_->now();
  target_pose_stamped.header.frame_id = "base";
  target_pose_stamped.pose.position.x = current_r_wrist_pose_.position.x;
  target_pose_stamped.pose.position.y = current_r_wrist_pose_.position.y;
  target_pose_stamped.pose.position.z = current_r_wrist_pose_.position.z;

  double big_ply_width = 0.75;
  double big_ply_length = 0.85;
  target_pose_stamped.pose.position.x -= big_ply_length;
  target_pose_stamped.pose.position.y -= big_ply_width*0.5;

  //for safety (to avoid hitting the table):
  if(target_pose_stamped.pose.position.z < 0.05) {
    target_pose_stamped.pose.position.z = 0.05;
  }

  target_pose_stamped.pose.orientation.x = starting_pose_wrist_following_ref_.orientation.x;
  target_pose_stamped.pose.orientation.y = starting_pose_wrist_following_ref_.orientation.y;
  target_pose_stamped.pose.orientation.z = starting_pose_wrist_following_ref_.orientation.z;
  target_pose_stamped.pose.orientation.w = starting_pose_wrist_following_ref_.orientation.w;

  //With current move_close_to_pick:
  target_pose_stamped.pose.orientation.x = 0.69805102;
  target_pose_stamped.pose.orientation.y = 0.71550464;
  target_pose_stamped.pose.orientation.z = -0.0189247;
  target_pose_stamped.pose.orientation.w = 0.02048730;

  bool success = set_reference_frame(target_pose_stamped);
  auto logger = node_->get_logger();
  if (success) {
    RCLCPP_INFO(logger, "Setting reference frame succeeded");
  } else {
    RCLCPP_INFO(logger, "Setting reference frame failed");
  }

  prev_r_wrist_pose_ = current_r_wrist_pose_;
  wrist_following_round_ref_ +=1;
}

bool OwnController::end_hand_guiding()
{
  auto logger = node_->get_logger();
  RCLCPP_INFO(node_->get_logger(), "Updating reference pose and increasing stiffness after hand guiding...");

  geometry_msgs::msg::Pose current_pose;
  bool success_to_get_pose = false;
  RCLCPP_INFO(logger, "Updating pose when ending hand guiding...");
  success_to_get_pose = get_pose_client_.update_pose(current_pose);

  if(!success_to_get_pose){
    RCLCPP_ERROR(logger, "Failed to update pose while ending hand guiding");
    return false;
  }

  RCLCPP_INFO(logger, "Pose updated");
  geometry_msgs::msg::PoseStamped current_pose_stamped;
  current_pose_stamped.header.stamp = node_->now();
  current_pose_stamped.header.frame_id = "base";
  current_pose_stamped.pose = current_pose;

  bool success_to_set_ref_pose = set_reference_frame(current_pose_stamped);
  if(!success_to_set_ref_pose){
    RCLCPP_ERROR(logger, "Failed to set ref pose while ending hand guiding");
    return false;
  }

  std::vector<double> new_stiffnesses(6);
  new_stiffnesses[0] = 0.2;
  new_stiffnesses[1] = 0.2;
  new_stiffnesses[2] = 0.2;
  new_stiffnesses[3] = 0.5;
  new_stiffnesses[4] = 0.5;
  new_stiffnesses[5] = 0.5;
  
  current_compliance_directions_.clear();
  
  if (!(rcm_manager_client_.set_stiffness(new_stiffnesses))) {
    return false;
  }
  current_stiffnesses_= new_stiffnesses;
  return true;
}

bool OwnController::activate_wrist_following_ref()
{
  geometry_msgs::msg::Pose starting_pose_wrist_following_ref_;
  auto logger = node_->get_logger();
  if(!get_pose_client_.update_pose(starting_pose_wrist_following_ref_)){
    RCLCPP_ERROR(logger, "Failed to update pose in the beginning of wrist following!");
    return false;
  }
  start_safe_tracking();
  return true;
}

bool OwnController::deactivate_wrist_following_ref()
{
  wrist_following_ref_activated_ = false;
  starting_pose_wrist_following_ref_ = geometry_msgs::msg::Pose{};
  return true;
}

bool OwnController::move_close_to_pick()
{
  std::vector<double> close_to_pick_joint_values = {-1.02672,0.83166,-0.34892,-1.56796,0.35808,2.39175,-2.33073};
  return joint_movement_client_.execute_joint_movement(close_to_pick_joint_values);
}

bool OwnController::approach_placing_point()
{
  geometry_msgs::msg::Pose goal_pose;
  goal_pose.position.x = 0.64293;
  goal_pose.position.y = -0.06159;
  goal_pose.position.z = 0.156590;
  goal_pose.orientation.x = 0.708063;
  goal_pose.orientation.y = 0.705085;
  goal_pose.orientation.z = 0.027606;
  goal_pose.orientation.w = -0.027178;

  //For test with obstacle:
  std::vector<geometry_msgs::msg::Pose> poses; 

  geometry_msgs::msg::Pose avoid_obstacle_pose;
  avoid_obstacle_pose.position.x = 0.22620;
  avoid_obstacle_pose.position.y = -0.37674;
  avoid_obstacle_pose.position.z = 0.27234;
  avoid_obstacle_pose.orientation.x = 0.75748;
  avoid_obstacle_pose.orientation.y = 0.65257;
  avoid_obstacle_pose.orientation.z = -0.00071;
  avoid_obstacle_pose.orientation.w = -0.01937;

  poses.push_back(avoid_obstacle_pose);
  poses.push_back(goal_pose);

  //Tests with obstacle:
  double timeout_ms=20000.0;
  double speed = 0.1;
  return cartesian_movement_client_.execute_cartesian_trajectory(poses, speed, timeout_ms);

}

bool OwnController::activate_wrist_following_servo()
{
  geometry_msgs::msg::Pose starting_pose_wrist_following_servo_;
  auto logger = node_->get_logger();
  if(!get_pose_client_.update_pose(starting_pose_wrist_following_servo_)){
    RCLCPP_ERROR(logger, "Failed to update pose in the beginning of wrist following!");
    return false;
  }
  wrist_following_servo_activated_ = true;
  return true;
}

bool OwnController::deactivate_wrist_following_servo()
{
  wrist_following_servo_activated_ = false;
  starting_pose_wrist_following_servo_ = geometry_msgs::msg::Pose{};
  return true;
}

bool OwnController::resist_more(std::string direction)
{
  std::vector<double> new_stiffnesses(6);
  new_stiffnesses = current_stiffnesses_;

  if (direction == "current") {
    for (std::string dir : current_compliance_directions_) {
      if (dir == "x") {
        new_stiffnesses[0] += stiffness_step_size_;
      } else if (dir == "y") {
        new_stiffnesses[1] += stiffness_step_size_;
      } else if (dir == "z") {
        new_stiffnesses[2] += stiffness_step_size_;
      } else if (dir == "rot_x") {
        new_stiffnesses[3] += stiffness_step_size_;
      } else if (dir == "rot_y") {
        new_stiffnesses[4] += stiffness_step_size_;
      } else if (dir == "rot_z") {
        new_stiffnesses[5] += stiffness_step_size_;
      }
    }
  } else if (direction == "all") {
    for (int i = 0; i < 6; i++) {
      new_stiffnesses[i] += stiffness_step_size_;
    }
  } else if (direction == "x") {
    new_stiffnesses[0] += stiffness_step_size_;
  } else if (direction == "y") {
    new_stiffnesses[1] += stiffness_step_size_;
  } else if (direction == "z") {
    new_stiffnesses[2] += stiffness_step_size_;
  } else if (direction == "rot_x") {
    new_stiffnesses[3] += stiffness_step_size_;
  } else if (direction == "rot_y") {
    new_stiffnesses[4] += stiffness_step_size_;
  } else if (direction == "rot_z") {
    new_stiffnesses[5] += stiffness_step_size_;
  } else {
    return false;
  }
  if (rcm_manager_client_.set_stiffness(new_stiffnesses)) {
    current_stiffnesses_= new_stiffnesses;
    return true;
  }
  return false;
}

bool OwnController::resist_less(std::string direction)
{
  std::vector<double> new_stiffnesses(6);
  new_stiffnesses = current_stiffnesses_;

  if (direction == "current") {
    for (std::string dir : current_compliance_directions_) {
      if (dir == "x") {
        new_stiffnesses[0] -= stiffness_step_size_;
      } else if (dir == "y") {
        new_stiffnesses[1] -= stiffness_step_size_;
      } else if (dir == "z") {
        new_stiffnesses[2] -= stiffness_step_size_;
      } else if (dir == "rot_x") {
        new_stiffnesses[3] -= stiffness_step_size_;
      } else if (dir == "rot_y") {
        new_stiffnesses[4] -= stiffness_step_size_;
      } else if (dir == "rot_z") {
        new_stiffnesses[5] -= stiffness_step_size_;
      }
    }
  } else if (direction == "all") {
    for (int i = 0; i < 6; i++) {
      new_stiffnesses[i] -= stiffness_step_size_;
    }
  } else if (direction == "x") {
    new_stiffnesses[0] -= stiffness_step_size_;
  } else if (direction == "y") {
    new_stiffnesses[1] -= stiffness_step_size_;
  } else if (direction == "z") {
    new_stiffnesses[2] -= stiffness_step_size_;
  } else if (direction == "rot_x") {
    new_stiffnesses[3] -= stiffness_step_size_;
  } else if (direction == "rot_y") {
    new_stiffnesses[4] -= stiffness_step_size_;
  } else if (direction == "rot_z") {
    new_stiffnesses[5] -= stiffness_step_size_;
  } else {
    return false;
  }
  if (rcm_manager_client_.set_stiffness(new_stiffnesses)) {
    current_stiffnesses_= new_stiffnesses;
    return true;
  }
  return false;
}

bool OwnController::set_reference_frame_to_current_ee_pose()
{
  geometry_msgs::msg::PoseStamped ref_frame_msg;
  geometry_msgs::msg::Pose reference_pose;
  bool communication_useful = false;
  auto logger = node_->get_logger();

  if(!get_pose_client_.update_pose(reference_pose)){
    RCLCPP_ERROR(logger, "Failed to update pose");
    return false;
  }
  ref_frame_msg.header.stamp = node_->get_clock()->now();
  ref_frame_msg.header.frame_id = "panda_link0";
  ref_frame_msg.pose.position.x = reference_pose.position.x;
  ref_frame_msg.pose.position.y = reference_pose.position.y;
  ref_frame_msg.pose.position.z = reference_pose.position.z;
  ref_frame_msg.pose.orientation.x = reference_pose.orientation.x;
  ref_frame_msg.pose.orientation.y = reference_pose.orientation.y;
  ref_frame_msg.pose.orientation.z = reference_pose.orientation.z;
  ref_frame_msg.pose.orientation.w = reference_pose.orientation.w;

  ref_frame_publisher_->publish(ref_frame_msg);
  updating_ref_pose_ = true;

  if (ref_pose_error_x_ > 0.01 || ref_pose_error_y_ > 0.01 || ref_pose_error_z_ > 0.01) {
    communication_useful = true;
    RCLCPP_INFO(logger, "Setting the reference frame. Please wait until ready.");
  }
  while (ref_pose_error_x_ > 0.01 || ref_pose_error_y_ > 0.01 || ref_pose_error_z_ > 0.01) {
    RCLCPP_INFO(logger, "Updating reference pose...");
    std::this_thread::sleep_for(250ms);
  }
  updating_ref_pose_ = false;
  if (communication_useful) {
    RCLCPP_INFO(logger, "Reference frame set.");
  }
  return true;
}

bool OwnController::set_reference_frame(geometry_msgs::msg::PoseStamped& reference_pose)
{
  ref_frame_publisher_->publish(reference_pose);
  updating_ref_pose_ = true;

  while ((ref_pose_error_x_ > 0.01 || ref_pose_error_y_ > 0.01 || ref_pose_error_z_ > 0.01) && wrist_following_ref_activated_ == false) {
    std::this_thread::sleep_for(250ms);
  }
  updating_ref_pose_ = false;

  if (wrist_following_ref_activated_ == false) {
  }
  return true;
}

bool OwnController::allow_compliance_3d()
{
  auto logger = node_->get_logger();
  if(!(switch_to_compliance_controller())) {
    RCLCPP_ERROR(logger, "Failed to activate compliance controller!");
    return false;
  }
 
  std::vector<double> new_stiffnesses(6);
  new_stiffnesses[0] = 0;
  new_stiffnesses[1] = 0;
  new_stiffnesses[2] = 0;
  new_stiffnesses[3] = default_stiffness_;
  new_stiffnesses[4] = default_stiffness_;
  new_stiffnesses[5] = default_stiffness_;
  
  current_compliance_directions_.clear();
  current_compliance_directions_.insert("x");
  current_compliance_directions_.insert("y");
  current_compliance_directions_.insert("z");
  
  if (!(rcm_manager_client_.set_stiffness(new_stiffnesses))) {
    return false;
  }
  current_stiffnesses_= new_stiffnesses;
  return true;
}

bool OwnController::disallow_compliance_3d()
{
  auto logger = node_->get_logger();
  if(!(switch_to_compliance_controller())) {
    RCLCPP_ERROR(logger, "Failed to activate compliance controller!");
    return false;
  }
  
  std::vector<double> new_stiffnesses(6);
  new_stiffnesses[0] = default_stiffness_;
  new_stiffnesses[1] = default_stiffness_;
  new_stiffnesses[2] = default_stiffness_;
  new_stiffnesses[3] = 0.0;
  new_stiffnesses[4] = 0.0;
  new_stiffnesses[5] = 0.0;
  
  for (std::string dir : all_compliance_directions_) {
    current_compliance_directions_.insert(dir);
  }
        
  current_compliance_directions_.erase("x");
  current_compliance_directions_.erase("y");
  current_compliance_directions_.erase("z");
  
  if (rcm_manager_client_.set_stiffness(new_stiffnesses)) {
    current_stiffnesses_= new_stiffnesses;
    return true;
  }
  return false;
}

bool OwnController::allow_compliance_6d()
{
  auto logger = node_->get_logger();
  if(!(switch_to_compliance_controller())) {
    RCLCPP_ERROR(logger, "Failed to activate compliance controller!");
    return false;
  }
  
  std::vector<double> new_stiffnesses(6);
  new_stiffnesses[0] = 0;
  new_stiffnesses[1] = 0;
  new_stiffnesses[2] = 0;
  new_stiffnesses[3] = 0;
  new_stiffnesses[4] = 0;
  new_stiffnesses[5] = 0;
  
  current_compliance_directions_.clear();
  current_compliance_directions_.insert("x");
  current_compliance_directions_.insert("y");
  current_compliance_directions_.insert("z");
  current_compliance_directions_.insert("rot_x");
  current_compliance_directions_.insert("rot_y");
  current_compliance_directions_.insert("rot_z");
  
  if (rcm_manager_client_.set_stiffness(new_stiffnesses)) {
    current_stiffnesses_= new_stiffnesses;
    return true;
  }
  return false;
}

bool OwnController::disallow_compliance_6d()
{
  auto logger = node_->get_logger();
  if(!(switch_to_compliance_controller())) {
    RCLCPP_ERROR(logger, "Failed to activate compliance controller!");
    return false;
  }
  
  std::vector<double> new_stiffnesses(6);
  new_stiffnesses[0] = default_stiffness_;
  new_stiffnesses[1] = default_stiffness_;
  new_stiffnesses[2] = default_stiffness_;
  new_stiffnesses[3] = default_stiffness_;
  new_stiffnesses[4] = default_stiffness_;
  new_stiffnesses[5] = default_stiffness_;
  
  current_compliance_directions_.clear();
  
  if (rcm_manager_client_.set_stiffness(new_stiffnesses)) {
    current_stiffnesses_= new_stiffnesses;
    return true;
  }
  return false;
}

bool OwnController::allow_compliance_plane()
{
  auto logger = node_->get_logger();
  if(!(switch_to_compliance_controller())) {
    RCLCPP_ERROR(logger, "Failed to activate compliance controller!");
    return false;
  }
  RCLCPP_INFO(logger, "Compliance controller successfully activated or already active!");
  
  std::vector<double> new_stiffnesses(6);
  new_stiffnesses[0] = 0.0;
  new_stiffnesses[1] = 0.0;
  new_stiffnesses[2] = default_stiffness_;
  new_stiffnesses[3] = default_stiffness_;
  new_stiffnesses[4] = default_stiffness_;
  new_stiffnesses[5] = default_stiffness_;
  
  current_compliance_directions_.clear();
  current_compliance_directions_.insert("x");
  current_compliance_directions_.insert("y");

  RCLCPP_INFO(logger, "Alive before rcm_manager_client call!");
  
  if (rcm_manager_client_.set_stiffness(new_stiffnesses)) {
    current_stiffnesses_= new_stiffnesses;
    RCLCPP_INFO(logger, "Stiffness set successfully!");

    return true;
  }
  RCLCPP_INFO(logger, "Alive after rcm_manager_client call, failed to set stiffness!");
  return false;
}

bool OwnController::disallow_compliance_plane()
{
  auto logger = node_->get_logger();
  if(!(switch_to_compliance_controller())) {
    RCLCPP_ERROR(logger, "Failed to activate compliance controller!");
    return false;
  }
  
  std::vector<double> new_stiffnesses(6);
  new_stiffnesses[0] = default_stiffness_;
  new_stiffnesses[1] = default_stiffness_;
  new_stiffnesses[2] = 0;
  new_stiffnesses[3] = 0;
  new_stiffnesses[4] = 0;
  new_stiffnesses[5] = 0;
  
  current_compliance_directions_.clear();
  current_compliance_directions_.insert("z");
  current_compliance_directions_.insert("rot_x");
  current_compliance_directions_.insert("rot_y");
  current_compliance_directions_.insert("rot_z");
  
  if (rcm_manager_client_.set_stiffness(new_stiffnesses)) {
    current_stiffnesses_= new_stiffnesses;
    return true;
  }
  return false;
}

bool OwnController::allow_compliance_also(std::string direction)
{
  auto logger = node_->get_logger();
  if(!(switch_to_compliance_controller())) {
    RCLCPP_ERROR(logger, "Failed to activate compliance controller!");
    return false;
  }
  
  std::vector<double> new_stiffnesses(6);
  new_stiffnesses = current_stiffnesses_;

  for (std::string current_dir : current_compliance_directions_) {
    if (current_dir == "x"){
      new_stiffnesses[0] = 0;
    } else if (current_dir == "y") {
      new_stiffnesses[1] = 0;
    } else if (current_dir == "z") {
      new_stiffnesses[2] = 0;
    } else if (current_dir == "rot_x") {
      new_stiffnesses[3] = 0;
    } else if (current_dir == "rot_y") {
      new_stiffnesses[4] = 0;
    } else if (current_dir == "rot_z") {
      new_stiffnesses[5] = 0;
    }
  }
  
  if (direction == "x") {
    new_stiffnesses[0] = 0;
  } else if (direction == "y") {
    new_stiffnesses[1] = 0;
  } else if (direction == "z") {
    new_stiffnesses[2] = 0;
  } else if (direction == "rot_x") {
    new_stiffnesses[3] = 0;
  } else if (direction == "rot_y") {
    new_stiffnesses[4] = 0;
  } else if (direction == "rot_z") {
    new_stiffnesses[5] = 0;
  } else {
    return false;
  }
  current_compliance_directions_.insert(direction);
  if (rcm_manager_client_.set_stiffness(new_stiffnesses)) {
    current_stiffnesses_= new_stiffnesses;
    return true;
  }
  return false;
}

bool OwnController::disallow_compliance_also(std::string direction)
{
  auto logger = node_->get_logger();
  if(!(switch_to_compliance_controller())) {
    RCLCPP_ERROR(logger, "Failed to activate compliance controller!");
    return false;
  }
  
  std::vector<double> new_stiffnesses(6);
  new_stiffnesses = current_stiffnesses_;

  for (std::string dir : all_compliance_directions_) {
    if (dir == "x") {
      if (current_compliance_directions_.find(dir) == current_compliance_directions_.end()) {
        new_stiffnesses[0] = default_stiffness_;
      } else {
        new_stiffnesses[0] = 0;
      }
    } else if (dir == "y") {
      if (current_compliance_directions_.find(dir) == current_compliance_directions_.end()) {
        new_stiffnesses[1] = default_stiffness_;
      } else {
        new_stiffnesses[1] = 0;
      }
    } else if (dir == "z") {
      if (current_compliance_directions_.find(dir) == current_compliance_directions_.end()) {
        new_stiffnesses[2] = default_stiffness_;
      } else {
        new_stiffnesses[2] = 0;
      }
    } else if (dir == "rot_x") {
      if (current_compliance_directions_.find(dir) == current_compliance_directions_.end()) {
        new_stiffnesses[3] = default_stiffness_;
      } else {
        new_stiffnesses[3] = 0;
      }
    } else if (dir == "rot_y") {
      if (current_compliance_directions_.find(dir) == current_compliance_directions_.end()) {
        new_stiffnesses[4] = default_stiffness_;
      } else {
        new_stiffnesses[4] = 0;
      }
    } else if (dir == "rot_z") {
      if (current_compliance_directions_.find(dir) == current_compliance_directions_.end()) {
        new_stiffnesses[5] = default_stiffness_;
      } else {
        new_stiffnesses[5] = 0;
      }
    }
  }
  if (direction == "x") {
    new_stiffnesses[0] = default_stiffness_;
  } else if (direction == "y") {
    new_stiffnesses[1] = default_stiffness_;
  } else if (direction == "z") {
    new_stiffnesses[2] = default_stiffness_;
  } else if (direction == "rot_x") {
    new_stiffnesses[3] = default_stiffness_;
  } else if (direction == "rot_y") {
    new_stiffnesses[4] = default_stiffness_;
  } else if (direction == "rot_z") {
    new_stiffnesses[5] = default_stiffness_;
  } else {
    return false;
  }
  current_compliance_directions_.erase(direction);
  
  if (rcm_manager_client_.set_stiffness(new_stiffnesses)) {
    current_stiffnesses_= new_stiffnesses;
    return true;
  }
  return false;
}

bool OwnController::allow_compliance_only(std::string direction)
{
  auto logger = node_->get_logger();
  if(!(switch_to_compliance_controller())) {
    RCLCPP_ERROR(logger, "Failed to activate compliance controller!");
    return false;
  }

  std::vector<double> new_stiffnesses(6);
  
  for (int i = 0; i < 6; i++) {
    new_stiffnesses[i] = default_stiffness_;
  }
  
  current_compliance_directions_.clear();

  if (direction == "x") {
    new_stiffnesses[0] = 0;
  } else if (direction == "y") {
    new_stiffnesses[1] = 0;
  }  else if (direction == "z") {
    new_stiffnesses[2] = 0;
  } else if (direction == "rot_x") {
    new_stiffnesses[3] = 0;
  } else if (direction == "rot_y") {
    new_stiffnesses[4] = 0;
  } else if (direction == "rot_z") {
    new_stiffnesses[5] = 0;
  } else {
    return false;
  }
  current_compliance_directions_.insert(direction);
  if (rcm_manager_client_.set_stiffness(new_stiffnesses)) {
    current_stiffnesses_= new_stiffnesses;
    return true;
  }
  return false;
}

bool OwnController::switch_to_compliance_controller()
{
  bool success = true;
  auto logger = node_->get_logger();
  if(!compliance_controller_activated_) {
    if(rcm_manager_client_.load_compliant_controller()) {
      compliance_controller_activated_ = true;
      success = true;
      current_stiffnesses_ = {default_stiffness_, default_stiffness_, default_stiffness_, default_stiffness_, default_stiffness_, default_stiffness_};
    } else {
      compliance_controller_activated_ = false;
      success = false;
    }
  } else {
    RCLCPP_INFO(logger, "Compliance controller already activated!");
    success = true;
  }
  return success;
}

bool OwnController::switch_to_default_controller()
{
  bool success = true;
  auto logger = node_->get_logger();
  if(compliance_controller_activated_) {
    success = rcm_manager_client_.load_default_controller();
    if (success) {
     compliance_controller_activated_ = false;
     return true;
    } else {
      compliance_controller_activated_ = true;
      return false;
    }
  } else {
    RCLCPP_INFO(logger, "Default controller already activated!");
    success = true;
  }
  return success;
}

bool OwnController::disallow_compliance_only(std::string direction)
{
  auto logger = node_->get_logger();
  if(!(switch_to_compliance_controller())) {
    RCLCPP_ERROR(logger, "Failed to activate compliance controller!");
    return false;
  }
  
  std::vector<double> new_stiffnesses = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  current_compliance_directions_.clear();

  if (direction == "x") {
    new_stiffnesses[0] = default_stiffness_;
  } else if (direction == "y") {
    new_stiffnesses[1] = default_stiffness_;
  }  else if (direction == "z") {
    new_stiffnesses[2] = default_stiffness_;
  } else if (direction == "rot_x") {
    new_stiffnesses[3] = default_stiffness_;
  } else if (direction == "rot_y") {
    new_stiffnesses[4] = default_stiffness_;
  } else if (direction == "rot_z") {
    new_stiffnesses[5] = default_stiffness_;
  } else {
    return false;
  }
  for (std::string dir:all_compliance_directions_) {
    current_compliance_directions_.insert(dir);
  }
  current_compliance_directions_.erase(direction);
  
  if (rcm_manager_client_.set_stiffness(new_stiffnesses)) {
    current_stiffnesses_= new_stiffnesses;
    return true;
  }
  return false;
}

bool OwnController::approach(const geometry_msgs::msg::Pose target_pose, double distance, bool advanced_mode){
  
  auto logger = node_->get_logger();

  geometry_msgs::msg::Pose approach_pose;
  if(!get_pose_client_.update_pose(approach_pose)){
    RCLCPP_ERROR(logger, "Failed to update pose");
    return false;
  }

  geometry_msgs::msg::Pose final_pose;

  std::vector<geometry_msgs::msg::Pose> poses; 

  if(advanced_mode){
    approach_pose = compensate_ee_orientation(translate_in_target_frame(target_pose, distance));
    final_pose = compensate_ee_orientation(target_pose);
  }
  else{
    approach_pose.position.x = target_pose.position.x;
    approach_pose.position.y = target_pose.position.y;
    approach_pose.position.z = target_pose.position.z + distance;

    approach_pose.orientation.x = default_ee_orientation_[0];
    approach_pose.orientation.y = default_ee_orientation_[1];
    approach_pose.orientation.z = default_ee_orientation_[2];
    approach_pose.orientation.w = default_ee_orientation_[3];

    final_pose = approach_pose;
    final_pose.position.z = target_pose.position.z;

  }

  std::this_thread::sleep_for(500ms);
  poses.push_back(approach_pose);
  if(!cartesian_movement_client_.execute_cartesian_trajectory(poses, 0.2)){
    RCLCPP_ERROR(logger, "Failed to move to approach position");
    return false;
  }

  poses.clear();
  poses.push_back(final_pose);
  std::this_thread::sleep_for(500ms);
  if(!cartesian_movement_client_.execute_cartesian_trajectory(poses, 0.1)){
    RCLCPP_ERROR(logger, "Failed to move to target position");
    return false;
  }

  return true;
}

bool OwnController::retreat(const geometry_msgs::msg::Pose starting_pose, const geometry_msgs::msg::Pose target_pose, double distance, bool advanced_mode){
  
  auto logger = node_->get_logger();
  std::vector<geometry_msgs::msg::Pose> poses; 
  geometry_msgs::msg::Pose retreat_pose = target_pose;

  if(advanced_mode){
    retreat_pose = compensate_ee_orientation(translate_in_target_frame(retreat_pose, distance));
  }
  else{
    retreat_pose.position.z += distance;
    retreat_pose.orientation.x = 0.;
    retreat_pose.orientation.y = 0.;
    retreat_pose.orientation.z = 0.;
    retreat_pose.orientation.w = 1.;
    retreat_pose = compensate_ee_orientation(retreat_pose);
  }
  geometry_msgs::msg::Pose rest_pose;
  rest_pose = retreat_pose;
  rest_pose.orientation.x = starting_pose.orientation.x;
  rest_pose.orientation.y = starting_pose.orientation.y;
  rest_pose.orientation.z = starting_pose.orientation.z;
  rest_pose.orientation.w = starting_pose.orientation.w;

  poses.push_back(retreat_pose);
  poses.push_back(rest_pose);
  std::this_thread::sleep_for(500ms);
  if(!cartesian_movement_client_.execute_cartesian_trajectory(poses, 0.1)){
    RCLCPP_ERROR(logger, "Failed to move to retreat position");
    return false;
  }

  return true;
}

geometry_msgs::msg::Pose OwnController::translate_in_target_frame(const geometry_msgs::msg::Pose target_pose, double distance){
  // Create a transform to the target frame
  tf2::Quaternion target_q(target_pose.orientation.x,
                        target_pose.orientation.y,
                        target_pose.orientation.z,
                        target_pose.orientation.w);
  
  tf2::Vector3 target_v(target_pose.position.x, target_pose.position.y, target_pose.position.z);

  tf2::Transform target_tf(target_q, target_v);
  tf2::Transform target_tf_inv = target_tf.inverse();
  
  geometry_msgs::msg::TransformStamped target_tf_msg = tf2::toMsg(tf2::Stamped(target_tf, 
     std::chrono::system_clock::now(), "target"));
  geometry_msgs::msg::TransformStamped target_tf_inv_msg = tf2::toMsg(tf2::Stamped(target_tf_inv, 
     std::chrono::system_clock::now(), "target_inv"));

  geometry_msgs::msg::Pose approach_pose;
  
  tf2::Vector3 translation_in_target_frame(0, 0, distance);
  tf2::Vector3 translation = target_tf * translation_in_target_frame;
  approach_pose = target_pose;
  approach_pose.position.x = translation.x();
  approach_pose.position.y = translation.y();
  approach_pose.position.z = translation.z();
  return approach_pose;
}

geometry_msgs::msg::Pose OwnController::compensate_ee_orientation(const geometry_msgs::msg::Pose target_pose){
  geometry_msgs::msg::Pose updated_pose = target_pose;

  tf2::Quaternion target_q(target_pose.orientation.x,
                        target_pose.orientation.y,
                        target_pose.orientation.z,
                        target_pose.orientation.w);
  
  tf2::Quaternion ee_q(default_ee_orientation_[0],
                      default_ee_orientation_[1],
                      default_ee_orientation_[2],
                      default_ee_orientation_[3]);

  tf2::Quaternion new_q = target_q * ee_q;
  new_q.normalize();

  updated_pose.orientation.x = new_q.x();
  updated_pose.orientation.y = new_q.y();
  updated_pose.orientation.z = new_q.z();
  updated_pose.orientation.w = new_q.w();

  return updated_pose;
}