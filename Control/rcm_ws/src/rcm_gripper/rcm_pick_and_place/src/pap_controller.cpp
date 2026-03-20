#include "rcm_pick_and_place/pap_controller.hpp"
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

PickAndPlaceController::PickAndPlaceController(std::shared_ptr<rclcpp::Node>& node)
: control_end_effector_client_(node), get_pose_client_(node), cartesian_movement_client_(node)
{
  node_ = node;
  io_name_ = {"gripper"};
  io_state_ = {0};
  width_ = 0.0;
  force_ = 0.5;
  angle_ = 0.0;

  std::vector<double> ee_orientation = {0, 0, 0, 1};

  node->declare_parameter("default_ee_orientation", ee_orientation);
  node->get_parameter("default_ee_orientation", default_ee_orientation_);

  bool use_current_orientation = false;
  node->declare_parameter("use_current_orientation", false);
  node->get_parameter("use_current_orientation", use_current_orientation);

  node->declare_parameter("direct_ee_poses", false);
  node->get_parameter("direct_ee_poses", direct_ee_poses_);

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

void PickAndPlaceController::set_force(double force){
  force_= force;
}

bool PickAndPlaceController::pick(const geometry_msgs::msg::Pose target_pose, double distance, bool advanced_mode){
  auto logger = node_->get_logger();

  RCLCPP_INFO(logger, "Picking sequence started");

  geometry_msgs::msg::Pose corrected_target_pose = target_pose;

  if(direct_ee_poses_) {
    corrected_target_pose = compensate_ee_orientation(corrected_target_pose, true);
  }

  geometry_msgs::msg::Pose starting_pose;
  if(!get_pose_client_.update_pose(starting_pose)){
    RCLCPP_ERROR(logger, "Failed to update pose");
    return false;
  }

  if(!approach(corrected_target_pose, distance, advanced_mode)){
    RCLCPP_ERROR(logger, "Approach failed");
    return false;
  }

  io_state_[0] = 1;
  if(!control_end_effector_client_.control_end_effector(io_name_, io_state_, width_, force_, angle_)){
    RCLCPP_ERROR(logger, "Grasping failed");
    return false;
  }

  if(!retreat(starting_pose, corrected_target_pose, distance, advanced_mode)){
    RCLCPP_ERROR(logger, "Retreat failed");
    return false;
  }

  RCLCPP_INFO(logger, "Picking sequence completed");

  return true;
}

bool PickAndPlaceController::place(const geometry_msgs::msg::Pose target_pose, double distance, bool advanced_mode){
  auto logger = node_->get_logger();

  RCLCPP_INFO(logger, "Placing sequence started");

  geometry_msgs::msg::Pose corrected_target_pose = target_pose;

  if(direct_ee_poses_) {
    corrected_target_pose = compensate_ee_orientation(corrected_target_pose, true);
  }

  geometry_msgs::msg::Pose starting_pose;
  if(!get_pose_client_.update_pose(starting_pose)){
    RCLCPP_ERROR(logger, "Failed to update pose");
    return false;
  }

  if(!approach(corrected_target_pose, distance, advanced_mode)){
    RCLCPP_ERROR(logger, "Approach failed");
    return false;
  }

  io_state_[0] = 0;
  if(!control_end_effector_client_.control_end_effector(io_name_, io_state_, width_, force_, angle_)){
    RCLCPP_ERROR(logger, "Releasing failed");
    return false;
  }

  if(!retreat(starting_pose, corrected_target_pose, distance, advanced_mode)){
    RCLCPP_ERROR(logger, "Retreat failed");
    return false;
  }

  RCLCPP_INFO(logger, "Placing sequence completed");

  return true;
}

bool PickAndPlaceController::approach(const geometry_msgs::msg::Pose target_pose, double distance, bool advanced_mode){
  
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
    // Basic method always approaches along z-axis
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

  // Delay movement slightly to allow the robot to settle down if it was previously moving
  std::this_thread::sleep_for(500ms);
  // Move to the approach position at a normal speed
  poses.push_back(approach_pose);
  if(!cartesian_movement_client_.execute_cartesian_trajectory(poses, 0.2)){
    RCLCPP_ERROR(logger, "Failed to move to approach position");
    return false;
  }

  poses.clear();
  poses.push_back(final_pose);

  // Delay movement slightly to allow the robot to settle down if it was previously moving
  std::this_thread::sleep_for(500ms);
  // Move slower towards the object
  if(!cartesian_movement_client_.execute_cartesian_trajectory(poses, 0.1)){
    RCLCPP_ERROR(logger, "Failed to move to target position");
    return false;
  }

  return true;
}

bool PickAndPlaceController::retreat(const geometry_msgs::msg::Pose starting_pose, const geometry_msgs::msg::Pose target_pose, double distance, bool advanced_mode){
  
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

  // Recovering original orientation before approach
  geometry_msgs::msg::Pose rest_pose;
  rest_pose = retreat_pose;
  rest_pose.orientation.x = starting_pose.orientation.x;
  rest_pose.orientation.y = starting_pose.orientation.y;
  rest_pose.orientation.z = starting_pose.orientation.z;
  rest_pose.orientation.w = starting_pose.orientation.w;

  poses.push_back(retreat_pose);
  poses.push_back(rest_pose);
  // Delay movement slightly to allow the robot to settle down if it was previously moving
  std::this_thread::sleep_for(500ms);
  // Slowly back away from the target position
  if(!cartesian_movement_client_.execute_cartesian_trajectory(poses, 0.1)){
    RCLCPP_ERROR(logger, "Failed to move to retreat position");
    return false;
  }

  return true;
}

geometry_msgs::msg::Pose PickAndPlaceController::translate_in_target_frame(const geometry_msgs::msg::Pose target_pose, double distance){
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

  // Transform target pose to target frame, save result to approach_pose
  //tf2::doTransform(target_pose, approach_pose, target_tf_msg);
  //approach_pose.position.z += distance;
  
  tf2::Vector3 translation_in_target_frame(0, 0, distance);
  tf2::Vector3 translation = target_tf * translation_in_target_frame;
  approach_pose = target_pose;
  approach_pose.position.x = translation.x();
  approach_pose.position.y = translation.y();
  approach_pose.position.z = translation.z();

  // Transform approach pose back to world frame
  //tf2::doTransform(approach_pose, approach_pose, target_tf_inv_msg);

  return approach_pose;
}

geometry_msgs::msg::Pose PickAndPlaceController::compensate_ee_orientation(const geometry_msgs::msg::Pose target_pose, bool inverse){
  geometry_msgs::msg::Pose updated_pose = target_pose;

  tf2::Quaternion target_q(target_pose.orientation.x,
                        target_pose.orientation.y,
                        target_pose.orientation.z,
                        target_pose.orientation.w);
  
  tf2::Quaternion ee_q(default_ee_orientation_[0],
                      default_ee_orientation_[1],
                      default_ee_orientation_[2],
                      default_ee_orientation_[3]);

  tf2::Quaternion new_q = target_q;

  if(inverse) {
    new_q *= ee_q.inverse();
  }
  else {
    new_q *= ee_q;
  }

  new_q.normalize();

  updated_pose.orientation.x = new_q.x();
  updated_pose.orientation.y = new_q.y();
  updated_pose.orientation.z = new_q.z();
  updated_pose.orientation.w = new_q.w();

  return updated_pose;
}