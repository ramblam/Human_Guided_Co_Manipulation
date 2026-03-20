#include <chrono>
#include <functional>
#include <memory>

#include <string>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/event_handler.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/server_goal_handle.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "rcm_cartesian_impedance_controller/cartesian_impedance_controller_ros.h"

#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace rcm_controllers
{
  /*! \brief Saturate a variable x with the limits x_min and x_max
  *
  * \param[in] x Value
  * \param[in] x_min Minimal value
  * \param[in] x_max Maximum value
  * \return Saturated value
  */
  double saturateValue(double x, double x_min, double x_max){
    return std::min(std::max(x, x_min), x_max);
  }

  /*! \brief Populates a wrench msg with value from Eigen vector
    *
    * It is assumed that the vector has the form transl_x, transl_y, transl_z, rot_x, rot_y, rot_z
    * \param[in] v Input vector
    * \param[out] wrench Wrench message
    */
  void EigenVectorToWrench(const Eigen::Matrix<double, 6, 1> &v, geometry_msgs::msg::Wrench *wrench){
    wrench->force.x = v(0);
    wrench->force.y = v(1);
    wrench->force.z = v(2);
    wrench->torque.x = v(3);
    wrench->torque.y = v(4);
    wrench->torque.z = v(5);
  }


  CartesianImpedanceControllerRos::CartesianImpedanceControllerRos()
  : controller_interface::ControllerInterface(), CartesianImpedanceController(), dof_(0){
  }

  controller_interface::InterfaceConfiguration
  CartesianImpedanceControllerRos::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    if (dof_ == 0) {
      fprintf(
        stderr,
        "During ros2_control interface configuration, degrees of freedom is not valid;"
        " it should be positive. Actual DOF is %zu\n",
        dof_);
      std::exit(EXIT_FAILURE);
    }
    conf.names.reserve(dof_ * params_.command_interfaces.size());
    for (const auto & joint_name : command_joint_names_) {
      for (const auto & interface_type : params_.command_interfaces){
        conf.names.push_back(joint_name + "/" + interface_type);
      }
    }
    return conf;
  }


  controller_interface::InterfaceConfiguration
  CartesianImpedanceControllerRos::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    conf.names.reserve(dof_ * params_.state_interfaces.size());

    for (const auto & joint_name : params_.joints) {
      for (const auto & interface_type : params_.state_interfaces) {
        conf.names.push_back(joint_name + "/" + interface_type);
      }
    }
    return conf;
  }


  controller_interface::return_type CartesianImpedanceControllerRos::update(const rclcpp::Time &time/*time*/, const rclcpp::Duration &period /*period*/){
    (void)time; // Unused parameter
    (void)period; // Unused parameter

    if (this->traj_running_)
    {
      trajUpdate();
    }

    updateParams();
    updateState();

    // Apply control law in base library
    calculateCommandedTorques();

    // Write commands
    for (size_t i = 0; i < this->n_joints_; ++i)
    {
      this->joint_command_interface_[3][i].get().set_value(this->tau_c_(i)); //this->joint_handles_[i].setCommand(this->tau_c_(i));
    }

    publishMsgsAndTf();

    return controller_interface::return_type::OK;
  }


  controller_interface::CallbackReturn CartesianImpedanceControllerRos::on_init(){
    try {
      // Create the parameter listener and get the parameters
      fprintf(stdout, "Not much to init.");
    }

    catch (const std::exception & e) {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }
  

  controller_interface::CallbackReturn CartesianImpedanceControllerRos::on_configure(
    const rclcpp_lifecycle::State & previous_state) {
    
    (void)previous_state; // Unused parameter

    auto logger = get_node()->get_logger();

    RCLCPP_INFO(this->get_node()->get_logger(), "Configuring CIC in namespace: %s",
                  this->get_node()->get_namespace());

    param_listener_ = std::make_shared<ParamListener>(get_node());

    // Verify that the parameter listner is still running
    if (!param_listener_)
    {
      RCLCPP_ERROR(logger, "Parameter listener not initialized!");
      return controller_interface::CallbackReturn::ERROR;
    }

    // get parameters from the listener in case they were updated
    params_ = param_listener_->get_params();

    // Default stiffness
    geometry_msgs::msg::Wrench cart_stiffness;
    cart_stiffness.force.x = params_.stiffness.force.x;
    cart_stiffness.force.y = params_.stiffness.force.y;
    cart_stiffness.force.z = params_.stiffness.force.z;
    cart_stiffness.torque.x = params_.stiffness.torque.x;
    cart_stiffness.torque.y = params_.stiffness.torque.y;
    cart_stiffness.torque.z = params_.stiffness.torque.z;
    double nullspace = params_.stiffness.nullspace;
    bool auto_damping = true;
    setStiffness(cart_stiffness, nullspace, auto_damping);

    // update internal parameters
    updateParams();

    // get degrees of freedom
    dof_ = params_.joints.size();

    // Internal configuration
    if(!initParams()) {
      RCLCPP_ERROR(logger, "Failed to initialize CIC parameters!");
      return controller_interface::CallbackReturn::ERROR;
    }

    if(!initRBDyn()) {
      RCLCPP_ERROR(logger, "Failed to initialize RBDyn!");
      return controller_interface::CallbackReturn::ERROR;
    }

    if(!initMessaging()) {
      RCLCPP_ERROR(logger, "Failed to initialize CIC messaging!");
      return controller_interface::CallbackReturn::ERROR;
    }

    if (params_.joints.empty())
    {
      // TODO(destogl): is this correct? Can we really move-on if no joint names are not provided?
      RCLCPP_WARN(logger, "'joints' parameter is empty.");
    }

    command_joint_names_ = params_.command_joints;

    if (command_joint_names_.empty())
    {
      command_joint_names_ = params_.joints;
      RCLCPP_INFO(
        logger, "No specific joint names are used for command interfaces. Using 'joints' parameter.");
    }
    else if (command_joint_names_.size() != params_.joints.size())
    {
      RCLCPP_ERROR(
        logger, "'command_joints' parameter has to have the same size as 'joints' parameter.");
      return CallbackReturn::FAILURE;
    }

    if (params_.command_interfaces.empty())
    {
      RCLCPP_ERROR(logger, "'command_interfaces' parameter is empty.");
      return CallbackReturn::FAILURE;
    }

    // Check if only allowed interface types are used and initialize storage to avoid memory
    // allocation during activation
    joint_command_interface_.resize(allowed_interface_types_.size());

    has_effort_command_interface_ =
      contains_interface_type(params_.command_interfaces, hardware_interface::HW_IF_EFFORT);

    // Check if state interfaces were specified
    if (params_.state_interfaces.empty())
    {
      RCLCPP_ERROR(logger, "'state_interfaces' parameter is empty.");
      return CallbackReturn::FAILURE;
    }

    if (!has_effort_command_interface_)
    {
      RCLCPP_ERROR(logger, "Effort interface not specified.");
      return CallbackReturn::FAILURE;
    }
    
    // Check if only allowed interface types are used and initialize storage to avoid memory
    // allocation during activation
    // Note: 'effort' storage is also here, but never used. Still, for this is OK.
    joint_state_interface_.resize(allowed_interface_types_.size());

    has_position_state_interface_ =
      contains_interface_type(params_.state_interfaces, hardware_interface::HW_IF_POSITION);
    has_velocity_state_interface_ =
      contains_interface_type(params_.state_interfaces, hardware_interface::HW_IF_VELOCITY);
    has_acceleration_state_interface_ =
      contains_interface_type(params_.state_interfaces, hardware_interface::HW_IF_ACCELERATION);

    // effort is always used alone so no need for size check
    if (
      has_effort_command_interface_ &&
      (!has_velocity_state_interface_ || !has_position_state_interface_))
    {
      RCLCPP_ERROR(
        logger,
        "'effort' command interface can only be used alone if 'velocity' and "
        "'position' state interfaces are present");
      return CallbackReturn::FAILURE;
    }

    auto get_interface_list = [](const std::vector<std::string> & interface_types)
    {
      std::stringstream ss_interfaces;
      for (size_t index = 0; index < interface_types.size(); ++index)
      {
        if (index != 0)
        {
          ss_interfaces << " ";
        }
        ss_interfaces << interface_types[index];
      }
      return ss_interfaces.str();
    };

    // Print output so users can be sure the interface setup is correct
    RCLCPP_INFO(
      logger, "Command interfaces are [%s] and state interfaces are [%s].",
      get_interface_list(params_.command_interfaces).c_str(),
      get_interface_list(params_.state_interfaces).c_str());

    return CallbackReturn::SUCCESS;

  }


  controller_interface::CallbackReturn CartesianImpedanceControllerRos::on_activate(
    const rclcpp_lifecycle::State & previous_state) {
    
    (void)previous_state; // Unused parameter

    auto logger = get_node()->get_logger();

    // get parameters from the listener in case they were updated
    params_ = param_listener_->get_params();

    // update internal parameters
    updateParams();

    // order all joints in the storage
    for (const auto & interface : params_.command_interfaces)
    {
      auto it =
        std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
      auto index = static_cast<size_t>(std::distance(allowed_interface_types_.begin(), it));
      if (!controller_interface::get_ordered_interfaces(
            command_interfaces_, command_joint_names_, interface, joint_command_interface_[index]))
      {
        RCLCPP_ERROR(
          logger, "Expected %zu '%s' command interfaces, got %zu.", dof_, interface.c_str(),
          joint_command_interface_[index].size());
        return CallbackReturn::ERROR;
      }
    }
    for (const auto & interface : params_.state_interfaces)
    {
      auto it =
        std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
      auto index = static_cast<size_t>(std::distance(allowed_interface_types_.begin(), it));
      if (!controller_interface::get_ordered_interfaces(
            state_interfaces_, params_.joints, interface, joint_state_interface_[index]))
      {
        RCLCPP_ERROR(
          logger, "Expected %zu '%s' state interfaces, got %zu.", dof_, interface.c_str(),
          joint_state_interface_[index].size());
        return CallbackReturn::ERROR;
      }
    }

    subscriber_is_active_ = true;
    last_state_publish_time_ = get_node()->now();

    // CIC internal activation
    tf_last_time_ = get_node()->get_clock()->now();
    starting();

    return CallbackReturn::SUCCESS;
  }


  controller_interface::CallbackReturn CartesianImpedanceControllerRos::on_deactivate(
    const rclcpp_lifecycle::State & previous_state) {
    
    (void)previous_state; // Unused parameter

    const auto active_goal = *rt_active_goal_.readFromNonRT();
    if (active_goal)
    {
      rt_has_pending_goal_.writeFromNonRT(false);
      auto action_res = std::make_shared<FollowJTrajAction::Result>();
      action_res->set__error_code(FollowJTrajAction::Result::INVALID_GOAL);
      action_res->set__error_string("Current goal cancelled during deactivate transition.");
      active_goal->setCanceled(action_res);
      rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
    }

    for (size_t index = 0; index < dof_; ++index)
    {
      // TODO(anyone): How to halt when using effort commands?
      if (has_effort_command_interface_)
      {
        joint_command_interface_[3][index].get().set_value(0.0);
      }
    }

    for (size_t index = 0; index < allowed_interface_types_.size(); ++index)
    {
      joint_command_interface_[index].clear();
      joint_state_interface_[index].clear();
    }
    release_interfaces();

    subscriber_is_active_ = false;

    //traj_external_point_ptr_.reset();

    return CallbackReturn::SUCCESS;
  }


  rclcpp_action::GoalResponse CartesianImpedanceControllerRos::goal_received_callback(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal){
    
    (void)uuid; // Unused parameter
    (void)goal; // Unused parameter

    RCLCPP_INFO(get_node()->get_logger(), "Received new goal from action server. (CIC)");

    // Precondition: Running controller
    if (get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Can't accept new action goals. Controller is not running.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(get_node()->get_logger(), "Accepted new action goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }


  rclcpp_action::CancelResponse CartesianImpedanceControllerRos::goal_cancelled_callback(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle){

    RCLCPP_INFO(get_node()->get_logger(), "Got request to cancel goal. (CIC)");

    // Check that cancel request refers to currently active goal (if any)
    const auto active_goal = *rt_active_goal_.readFromNonRT();
    if (active_goal && active_goal->gh_ == goal_handle) {
      RCLCPP_INFO(
        get_node()->get_logger(), "Canceling active action goal because cancel callback received.");

      // Mark the current goal as canceled
      auto action_res = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
      active_goal->setCanceled(action_res);
      rt_active_goal_.writeFromNonRT(std::shared_ptr<realtime_tools::RealtimeServerGoalHandle<control_msgs::action::FollowJointTrajectory>>());

      this->traj_running_ = false;
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }


  void CartesianImpedanceControllerRos::goal_accepted_callback(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle) {

    // Update new trajectory
    {
      preempt_active_goal();
      auto traj_msg = goal_handle->get_goal()->trajectory;

      trajStart(traj_msg);
    }

    // Update the active goal
    std::shared_ptr<realtime_tools::RealtimeServerGoalHandle<control_msgs::action::FollowJointTrajectory>> rt_goal = std::make_shared<realtime_tools::RealtimeServerGoalHandle<control_msgs::action::FollowJointTrajectory>>(goal_handle);
    rt_goal->preallocated_feedback_->joint_names = params_.joints;
    rt_goal->execute();
    rt_active_goal_.writeFromNonRT(rt_goal);

    // Set smartpointer to expire for create_wall_timer to delete previous entry from timer list
    goal_handle_timer_.reset();

    // Setup goal status checking timer
    goal_handle_timer_ = get_node()->create_wall_timer(
      action_monitor_period_.to_chrono<std::chrono::nanoseconds>(),
      std::bind(&realtime_tools::RealtimeServerGoalHandle<control_msgs::action::FollowJointTrajectory>::runNonRealtime, rt_goal));
  }


  void CartesianImpedanceControllerRos::preempt_active_goal() {
    const auto active_goal = *rt_active_goal_.readFromNonRT();
    if (active_goal)
    {
      auto action_res = std::make_shared<FollowJTrajAction::Result>();
      action_res->set__error_code(FollowJTrajAction::Result::INVALID_GOAL);
      action_res->set__error_string("Current goal cancelled due to new incoming action.");
      active_goal->setCanceled(action_res);
      rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
    }
  }

  void CartesianImpedanceControllerRos::topic_callback(const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> msg){
    preempt_active_goal();
    auto traj_msg = *msg;
    trajExecute(traj_msg);
  }

  bool CartesianImpedanceControllerRos::contains_interface_type(
    const std::vector<std::string> & interface_type_list, const std::string & interface_type){
    return std::find(interface_type_list.begin(), interface_type_list.end(), interface_type) !=
          interface_type_list.end();
  }


  bool CartesianImpedanceControllerRos::initMessaging(){

    auto node = get_node();
    auto logger = node->get_logger();

    // tf2 setup
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    // parameter subscriptions
    sub_damping_factors_ = node->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "set_damping_factors", 1, std::bind(&CartesianImpedanceControllerRos::cartesianDampingFactorCb, this, std::placeholders::_1));
    
    sub_cartesian_stiffness_ = node->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "set_cartesian_stiffness", 1, std::bind(&CartesianImpedanceControllerRos::cartesianStiffnessCb, this, std::placeholders::_1));

    sub_controller_config_ = node->create_subscription<rcm_cartesian_impedance_controller::msg::ControllerConfig>(
    "set_config", 1, std::bind(&CartesianImpedanceControllerRos::controllerConfigCb, this, std::placeholders::_1));

    sub_reference_pose_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "reference_pose", 1, std::bind(&CartesianImpedanceControllerRos::referencePoseCb, this, std::placeholders::_1));
    
    sub_wrench_command_ = node->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "set_cartesian_wrench", 1, std::bind(&CartesianImpedanceControllerRos::wrenchCommandCb, this, std::placeholders::_1));

    // state publishers
    pub_torques_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("commanded_torques", 10);
    pub_state_ = node->create_publisher<rcm_cartesian_impedance_controller::msg::ControllerState>("controller_state", 10);

    // trajectory command interfaces
    joint_command_subscriber_ =
      node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        std::string(node->get_name()) + "/joint_trajectory", rclcpp::SystemDefaultsQoS(),
        std::bind(&CartesianImpedanceControllerRos::topic_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      logger, "Action status changes will be monitored at %.2f Hz.", params_.action_monitor_rate);
    action_monitor_period_ = rclcpp::Duration::from_seconds(1.0 / params_.action_monitor_rate);

    using namespace std::placeholders;
    action_server_ = rclcpp_action::create_server<FollowJTrajAction>(
      node->get_node_base_interface(), node->get_node_clock_interface(),
      node->get_node_logging_interface(),node->get_node_waitables_interface(),
      std::string(node->get_name()) + "/follow_joint_trajectory",
      std::bind(&CartesianImpedanceControllerRos::goal_received_callback, this, _1, _2),
      std::bind(&CartesianImpedanceControllerRos::goal_cancelled_callback, this, _1),
      std::bind(&CartesianImpedanceControllerRos::goal_accepted_callback, this, _1));

    return true;
  }


  bool CartesianImpedanceControllerRos::initRBDyn(){
    auto node = this->get_node();
    auto logger = node->get_logger();

    // Get the URDF XML from a parameter or topic
    if(params_.robot_description.empty()){
      RCLCPP_INFO(logger, "No URDF file given");
      
      // Subscribing to the /robot_description topic
      sub_robot_description_ = node->create_subscription<std_msgs::msg::String>(
      "robot_description", rclcpp::QoS(1).transient_local(), std::bind(&CartesianImpedanceControllerRos::robotDescriptionCb, this, std::placeholders::_1));
      
      for(int c=0; c<20; c++){
        RCLCPP_INFO_ONCE(logger, "Waiting for URDF from /robot_description");
        rclcpp::sleep_for(250ms);

        std::lock_guard<std::mutex> lock(robot_description_mutex_);
        if(!robot_description_buffer_.empty()){
          robot_description_ = robot_description_buffer_;
          RCLCPP_INFO(logger, "Received URDF from /robot_description");
          break;
        }
      }

    }
    else {
      robot_description_ = params_.robot_description;
    }

    if(robot_description_.empty()){
      RCLCPP_ERROR(logger, "Robot description is empty.");
      return false;
    }

    try
    {
      rbdyn_wrapper_.init_rbdyn(robot_description_, end_effector_);
    }
    
    catch (std::runtime_error &e)
    {
      RCLCPP_ERROR(logger, "Error when intializing RBDyn: %s", e.what());
      return false;
    }
    RCLCPP_INFO_STREAM(logger, "Number of joints found in urdf: " << rbdyn_wrapper_.n_joints());
    if (size_t(rbdyn_wrapper_.n_joints()) < n_joints_)
    {
      RCLCPP_ERROR(logger, "Number of joints in the URDF is smaller than supplied number of joints. %i < %zu", rbdyn_wrapper_.n_joints(), n_joints_);
      return false;
    }
    else if (size_t(rbdyn_wrapper_.n_joints()) > n_joints_)
    {
      RCLCPP_WARN(logger, "Number of joints in the URDF is greater than supplied number of joints: %i > %zu. Assuming that the actuated joints come first.", rbdyn_wrapper_.n_joints(), n_joints_);
    }

    root_frame_ = rbdyn_wrapper_.root_link();
    node->declare_parameter("root_frame", root_frame_);

    RCLCPP_INFO(logger, "Finished RBDyn initialization.");

    return true;
  }


  bool CartesianImpedanceControllerRos::initParams(){
    // General config
    this->end_effector_ = params_.end_effector;
    RCLCPP_INFO(this->get_node()->get_logger(), "End effector link is: %s", this->end_effector_.c_str());

    this->wrench_ee_frame_ = params_.wrench_ee_frame;
    this->delta_tau_max_ = params_.delta_tau_max;
    this->update_frequency_ = params_.update_frequency;
    
    // Filtering
    RCLCPP_INFO(this->get_node()->get_logger(), "Reading filtering configuration...");
    this->filter_params_nullspace_config_ = params_.filtering.nullspace_config;
    this->filter_params_stiffness_ = params_.filtering.stiffness;
    this->filter_params_pose_ = params_.filtering.pose;
    this->filter_params_wrench_ = params_.filtering.wrench;

    // Verbosity
    RCLCPP_INFO(this->get_node()->get_logger(), "Reading verbosity configuration...");
    this->verbose_print_ = params_.verbosity.print;
    this->verbose_state_ = params_.verbosity.state_msgs;
    this->verbose_tf_ = params_.verbosity.tf_frames;

    // Initialize base_tools and member variables
    RCLCPP_INFO(this->get_node()->get_logger(), "Detecting number of joints...");

    this->setNumberOfJoints(params_.joints.size());
    if (this->n_joints_ < 6)
    {
      RCLCPP_WARN(this->get_node()->get_logger(), "Number of joints is below 6. Functions might be limited.");
    }
    if (this->n_joints_ < 7)
    {
      RCLCPP_WARN(this->get_node()->get_logger(), "Number of joints is below 7. No redundant joint for nullspace.");
    }
    this->tau_m_ = Eigen::VectorXd(this->n_joints_);

    // Update dynamic parameters
    updateParams();

    RCLCPP_INFO(this->get_node()->get_logger(), "Finished Parameter initialization.");
    return true;
  }


  void CartesianImpedanceControllerRos::starting(){
    this->updateState();

    // Set reference pose to current pose and q_d_nullspace
    this->initDesiredPose(this->position_, this->orientation_);
    this->initNullspaceConfig(this->q_);
    RCLCPP_INFO(this->get_node()->get_logger(), "Started Cartesian Impedance Controller");
  }


  bool CartesianImpedanceControllerRos::getFk(const Eigen::VectorXd &q, Eigen::Vector3d *position,
                                              Eigen::Quaterniond *orientation) const{
    rbdyn_wrapper::EefState ee_state;
    // If the URDF contains more joints than there are controlled, only the state of the controlled ones are known
    if (size_t(this->rbdyn_wrapper_.n_joints()) != this->n_joints_)
    {
      Eigen::VectorXd q_rb = Eigen::VectorXd::Zero(this->rbdyn_wrapper_.n_joints());
      q_rb.head(q.size()) = q;
      ee_state = this->rbdyn_wrapper_.perform_fk(q_rb);
    }
    else
    {
      ee_state = this->rbdyn_wrapper_.perform_fk(q);
    }
    *position = ee_state.translation;
    *orientation = ee_state.orientation;
    return true;
  }


  bool CartesianImpedanceControllerRos::getJacobian(const Eigen::VectorXd &q, const Eigen::VectorXd &dq,
                                                    Eigen::MatrixXd *jacobian){
    // If the URDF contains more joints than there are controlled, only the state of the controlled ones are known
    if (size_t(this->rbdyn_wrapper_.n_joints()) != this->n_joints_)
    {
      Eigen::VectorXd q_rb = Eigen::VectorXd::Zero(this->rbdyn_wrapper_.n_joints());
      q_rb.head(q.size()) = q;
      Eigen::VectorXd dq_rb = Eigen::VectorXd::Zero(this->rbdyn_wrapper_.n_joints());
      dq_rb.head(dq.size()) = dq;
      *jacobian = this->rbdyn_wrapper_.jacobian(q_rb, dq_rb);
    }
    else
    {
      *jacobian = this->rbdyn_wrapper_.jacobian(q, dq);
    }
    *jacobian = jacobian_perm_ * *jacobian;
    return true;
  }


  void CartesianImpedanceControllerRos::updateState(){
    for (size_t i = 0; i < this->n_joints_; ++i) {
      this->q_[i] = joint_state_interface_[0][i].get().get_value(); //this->q_[i] = this->joint_handles_[i].getPosition();
      this->dq_[i] = joint_state_interface_[1][i].get().get_value(); //this->dq_[i] = this->joint_handles_[i].getVelocity();
      this->tau_m_[i] = joint_state_interface_[3][i].get().get_value(); //this->tau_m_[i] = this->joint_handles_[i].getEffort();
    }
    getJacobian(this->q_, this->dq_, &this->jacobian_);
    getFk(this->q_, &this->position_, &this->orientation_);
  }

  void CartesianImpedanceControllerRos::setDampingFactors(const geometry_msgs::msg::Wrench &cart_damping, double nullspace){
    CartesianImpedanceController::setDampingFactors(saturateValue(cart_damping.force.x, dmp_factor_min_, dmp_factor_max_),
                                             saturateValue(cart_damping.force.y, dmp_factor_min_, dmp_factor_max_),
                                             saturateValue(cart_damping.force.z, dmp_factor_min_, dmp_factor_max_),
                                             saturateValue(cart_damping.torque.x, dmp_factor_min_, dmp_factor_max_),
                                             saturateValue(cart_damping.torque.y, dmp_factor_min_, dmp_factor_max_),
                                             saturateValue(cart_damping.torque.z, dmp_factor_min_, dmp_factor_max_),
                                             saturateValue(nullspace, dmp_factor_min_, dmp_factor_max_));
  }


  void CartesianImpedanceControllerRos::setStiffness(const geometry_msgs::msg::Wrench &cart_stiffness, double nullspace, bool auto_damping){
    CartesianImpedanceController::setStiffness(saturateValue(cart_stiffness.force.x, trans_stf_min_, trans_stf_max_),
                                               saturateValue(cart_stiffness.force.y, trans_stf_min_, trans_stf_max_),
                                               saturateValue(cart_stiffness.force.z, trans_stf_min_, trans_stf_max_),
                                               saturateValue(cart_stiffness.torque.x, rot_stf_min_, rot_stf_max_),
                                               saturateValue(cart_stiffness.torque.y, rot_stf_min_, rot_stf_max_),
                                               saturateValue(cart_stiffness.torque.z, rot_stf_min_, rot_stf_max_),
                                               saturateValue(nullspace, ns_min_, ns_max_), auto_damping);
  }


  bool CartesianImpedanceControllerRos::transformWrench(Eigen::Matrix<double, 6, 1> *cartesian_wrench,
                                                        const std::string &from_frame, const std::string &to_frame) const{
    try
    {
      tf2::Stamped<tf2::Transform> transform;
      tf2::fromMsg(tf_buffer_->lookupTransform(to_frame, from_frame, tf2::TimePointZero), transform);
      tf2::Vector3 v_f(cartesian_wrench->operator()(0), cartesian_wrench->operator()(1), cartesian_wrench->operator()(2));
      tf2::Vector3 v_t(cartesian_wrench->operator()(3), cartesian_wrench->operator()(4), cartesian_wrench->operator()(5));
      tf2::Vector3 v_f_rot = tf2::quatRotate(transform.getRotation(), v_f);
      tf2::Vector3 v_t_rot = tf2::quatRotate(transform.getRotation(), v_t);
      *cartesian_wrench << v_f_rot[0], v_f_rot[1], v_f_rot[2], v_t_rot[0], v_t_rot[1], v_t_rot[2];
      return true;
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_ERROR(this->get_node()->get_logger(), "%s", ex.what());
      return false;
    }
  }

  void CartesianImpedanceControllerRos::publishMsgsAndTf(){
  
    /* This function is based on a later ROS2 fork by Cheng Tang, which is why a separate copyright notice is provided here
    *
    * Copyright 2022 Matthias Mayr, 2025 Cheng Tang
    *
    * Redistribution and use in source and binary forms, with or without modification, are permitted provided that 
    * the following conditions are met:
    *
    * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    *
    * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer
    *    in the documentation and/or other materials provided with the distribution.
    *
    *
    * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived
    *    from this software without specific prior written permission.
    *
    *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
    *  BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
    *  SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
    *  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
    *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
    */    
    auto node = get_node();
    auto commanded_torques_msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
    commanded_torques_msg->data.resize(this->n_joints_);
    for (size_t i = 0; i < this->n_joints_; i++){
        commanded_torques_msg->data[i] = this->tau_c_(i);
    }

    this->pub_torques_->publish(*commanded_torques_msg);
    const Eigen::Matrix<double, 6, 1> error{this->getPoseError()};

    if (this->verbose_print_){
        RCLCPP_INFO(node->get_logger(), "\nCartesian Position:\n%f %f %f\nError:\n%f %f %f\nCartesian Stiffness:\n%f %f %f\nCartesian damping:\n%f %f %f\nNullspace stiffness:\n%f\nq_d_nullspace:\n%f\n",
                    this->position_(0), this->position_(1), this->position_(2), error(0), error(1), error(2),
                    this->cartesian_stiffness_(0, 0), this->cartesian_stiffness_(1, 1), this->cartesian_stiffness_(2, 2),
                    this->cartesian_damping_(0, 0), this->cartesian_damping_(1, 1), this->cartesian_damping_(2, 2),
                    this->nullspace_stiffness_, this->q_d_nullspace_(0));
    }

    if (this->verbose_tf_ && (node->now() - this->tf_last_time_).seconds() > 1.0){
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = node->now();
        transform_stamped.header.frame_id = this->root_frame_;
        transform_stamped.child_frame_id = this->end_effector_ + "_ee_fk";
        tf2::Transform tf_transform(tf2::Quaternion(this->orientation_.x(), this->orientation_.y(), this->orientation_.z(), this->orientation_.w()),
                                    tf2::Vector3(this->position_.x(), this->position_.y(), this->position_.z()));
        transform_stamped.transform.translation.x = tf_transform.getOrigin().x();
        transform_stamped.transform.translation.y = tf_transform.getOrigin().y();
        transform_stamped.transform.translation.z = tf_transform.getOrigin().z();
        transform_stamped.transform.rotation.x = tf_transform.getRotation().x();
        transform_stamped.transform.rotation.y = tf_transform.getRotation().y();
        transform_stamped.transform.rotation.z = tf_transform.getRotation().z();
        transform_stamped.transform.rotation.w = tf_transform.getRotation().w();
        tf_broadcaster_->sendTransform(transform_stamped);
        transform_stamped.child_frame_id = this->end_effector_ + "_ee_ref_pose";
        tf2::Transform tf_transform_d(tf2::Quaternion(this->orientation_d_.x(), this->orientation_d_.y(), this->orientation_d_.z(), this->orientation_d_.w()),
                                      tf2::Vector3(this->position_d_.x(), this->position_d_.y(), this->position_d_.z()));
        transform_stamped.transform.translation.x = tf_transform_d.getOrigin().x();
        transform_stamped.transform.translation.y = tf_transform_d.getOrigin().y();
        transform_stamped.transform.translation.z = tf_transform_d.getOrigin().z();
        transform_stamped.transform.rotation.x = tf_transform_d.getRotation().x();
        transform_stamped.transform.rotation.y = tf_transform_d.getRotation().y();
        transform_stamped.transform.rotation.z = tf_transform_d.getRotation().z();
        transform_stamped.transform.rotation.w = tf_transform_d.getRotation().w();
        tf_broadcaster_->sendTransform(transform_stamped);
        this->tf_last_time_ = node->now();
      }

    if (this->verbose_state_){
        auto state_msg = std::make_shared<rcm_cartesian_impedance_controller::msg::ControllerState>();
        state_msg->header.stamp = node->now();
        state_msg->current_pose.position = tf2::toMsg(this->position_);
        state_msg->current_pose.orientation = tf2::toMsg(this->orientation_);
        state_msg->reference_pose.position = tf2::toMsg(this->position_d_);
        state_msg->reference_pose.orientation = tf2::toMsg(this->orientation_d_);
        Eigen::Vector3d position_error = error.head(3);
        state_msg->pose_error.position = tf2::toMsg(position_error);
        Eigen::Quaterniond q = Eigen::AngleAxisd(error(3), Eigen::Vector3d::UnitX()) *
                              Eigen::AngleAxisd(error(4), Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(error(5), Eigen::Vector3d::UnitZ());
        state_msg->pose_error.orientation = tf2::toMsg(q);
        EigenVectorToWrench(this->cartesian_stiffness_.diagonal(), &state_msg->cartesian_stiffness);
        EigenVectorToWrench(this->cartesian_damping_.diagonal(), &state_msg->cartesian_damping);
        EigenVectorToWrench(this->getAppliedWrench(), &state_msg->commanded_wrench);

        for (size_t i = 0; i < this->n_joints_; i++){
            state_msg->joint_state.position.at(i) = this->q_(i);
            state_msg->joint_state.velocity.at(i) = this->dq_(i);
            state_msg->joint_state.effort.at(i) = this->tau_m_(i);
            state_msg->commanded_torques.at(i) = this->tau_c_(i);
            state_msg->nullspace_config.at(i) = this->q_d_nullspace_(i);
            state_msg->commanded_torques.at(i) = this->tau_c_(i);
        }

        state_msg->nullspace_stiffness = this->nullspace_stiffness_;
        state_msg->nullspace_damping = this->nullspace_damping_;
        const Eigen::Matrix<double, 6, 1> dx = this->jacobian_ * this->dq_;
        state_msg->cartesian_velocity = sqrt(dx(0) * dx(0) + dx(1) * dx(1) + dx(2) * dx(2));
        this->pub_state_->publish(*state_msg);
    }
  }


  void CartesianImpedanceControllerRos::trajExecute(const trajectory_msgs::msg::JointTrajectory &trajectory){
    this->traj_duration_ = trajectory.points[trajectory.points.size() - 1].time_from_start;

    if(trajectory.points.size() > 1){
      RCLCPP_WARN_STREAM(this->get_node()->get_logger(), "Trajectory has length " << trajectory.points.size() << ", but only the last point can be executed using this interface.");
    }

    RCLCPP_INFO_STREAM(this->get_node()->get_logger(), "Starting a new trajectory that takes " << (this->traj_duration_).seconds() << "s.");
    this->trajectory_ = trajectory;
    this->traj_start_ = this->get_node()->get_clock()->now();
    this->traj_index_ = trajectory.points.size()-1;

    if (this->nullspace_stiffness_ < 5.)
    {
      RCLCPP_WARN(this->get_node()->get_logger(), "Nullspace stiffness is low. The joints might not follow the planned path.");
    }

    RCLCPP_INFO_STREAM(this->get_node()->get_logger(), "Updating active trajectory point!");
    // Get end effector pose

    Eigen::VectorXd q = Eigen::VectorXd::Map(trajectory_.points.at(this->traj_index_).positions.data(),
                                            trajectory_.points.at(this->traj_index_).positions.size());
    if (this->verbose_print_)
    {
      RCLCPP_INFO_STREAM(this->get_node()->get_logger(), "Index " << this->traj_index_ << " q_nullspace: " << q.transpose());
    }
    // Update end-effector pose and nullspace
    getFk(q, &this->position_d_target_, &this->orientation_d_target_);
    setNullspaceConfig(q);

  }

  void CartesianImpedanceControllerRos::trajStart(const trajectory_msgs::msg::JointTrajectory &trajectory){
    this->traj_duration_ = trajectory.points[trajectory.points.size() - 1].time_from_start;
    RCLCPP_INFO_STREAM(this->get_node()->get_logger(), "Starting a new trajectory with " << trajectory.points.size() << " points that takes " << (this->traj_duration_).seconds() << "s.");
    this->trajectory_ = trajectory;
    this->traj_running_ = true;
    this->traj_start_ = this->get_node()->get_clock()->now();
    this->traj_index_ = 0;
    trajUpdate();
    if (this->nullspace_stiffness_ < 5.)
    {
      RCLCPP_WARN(this->get_node()->get_logger(), "Nullspace stiffness is low. The joints might not follow the planned path.");
    }
  }


  void CartesianImpedanceControllerRos::trajUpdate() {

    if (this->get_node()->get_clock()->now() > (this->traj_start_ + trajectory_.points.at(this->traj_index_).time_from_start))
    {
      RCLCPP_INFO_STREAM(this->get_node()->get_logger(), "Updating active trajectory point!");
      // Get end effector pose

      Eigen::VectorXd q = Eigen::VectorXd::Map(trajectory_.points.at(this->traj_index_).positions.data(),
                                               trajectory_.points.at(this->traj_index_).positions.size());
      if (this->verbose_print_)
      {
        RCLCPP_INFO_STREAM(this->get_node()->get_logger(), "Index " << this->traj_index_ << " q_nullspace: " << q.transpose());
      }
      // Update end-effector pose and nullspace
      getFk(q, &this->position_d_target_, &this->orientation_d_target_);
      this->setNullspaceConfig(q);
      
      this->traj_index_++;
    }

    if (this->get_node()->get_clock()->now() > (this->traj_start_ + this->traj_duration_))
    {
      RCLCPP_INFO_STREAM(this->get_node()->get_logger(), "Finished executing trajectory.");
      
      
      const auto active_goal = *rt_active_goal_.readFromRT();
      auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
      result->set__error_code(control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL);
      result->set__error_string("Goal successfully reached!");
      active_goal->setSucceeded(result);
      // TODO(matthew-reynolds): Need a lock-free write here
      // See https://github.com/ros-controls/ros2_controllers/issues/168
      rt_active_goal_.writeFromNonRT(std::shared_ptr<realtime_tools::RealtimeServerGoalHandle<control_msgs::action::FollowJointTrajectory>>());
    
      RCLCPP_INFO(get_node()->get_logger(), "Goal reached, success!");
      
      this->traj_running_ = false;
      
    }
  }


  void CartesianImpedanceControllerRos::robotDescriptionCb(const std_msgs::msg::String &msg){
    RCLCPP_INFO(get_node()->get_logger(), "Received new URDF!");
    std::lock_guard<std::mutex> lock(robot_description_mutex_);
    robot_description_buffer_ = std::string(msg.data);
  }

  void CartesianImpedanceControllerRos::cartesianDampingFactorCb(const geometry_msgs::msg::WrenchStamped &msg){
    RCLCPP_INFO(get_node()->get_logger(), "Received damping factor change message!");
    this->setDampingFactors(msg.wrench, this->damping_factors_[6]);
  }


  void CartesianImpedanceControllerRos::cartesianStiffnessCb(const geometry_msgs::msg::WrenchStamped &msg){
    RCLCPP_INFO(get_node()->get_logger(), "Received stiffness change message!");
    this->setStiffness(msg.wrench, this->nullspace_stiffness_target_);
  }
  

  void CartesianImpedanceControllerRos::controllerConfigCb(const rcm_cartesian_impedance_controller::msg::ControllerConfig &msg){
    this->setStiffness(msg.cartesian_stiffness, msg.nullspace_stiffness, false);
    this->setDampingFactors(msg.cartesian_damping_factors, msg.nullspace_damping_factor);

    if (msg.q_d_nullspace.size() == this->n_joints_)
    {
      Eigen::VectorXd q_d_nullspace(this->n_joints_);
      for (size_t i = 0; i < this->n_joints_; i++)
      {
        q_d_nullspace(i) = msg.q_d_nullspace.at(i);
      }
      this->setNullspaceConfig(q_d_nullspace);
    }
    else
    {
      RCLCPP_WARN(this->get_node()->get_logger(), "Nullspace configuration does not have the correct amount of entries. Got %ld, expected %ld. Ignoring.",
                  msg.q_d_nullspace.size(), this->n_joints_);
    
    }
  }


  void CartesianImpedanceControllerRos::referencePoseCb(const geometry_msgs::msg::PoseStamped &msg){
    if (!msg.header.frame_id.empty() && msg.header.frame_id != this->root_frame_)
    {
      RCLCPP_WARN(this->get_node()->get_logger(), "Reference poses need to be in the root frame '%s'. Ignoring.", this->root_frame_.c_str());
      return;
    }
    Eigen::Vector3d position_d;
    position_d << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
    const Eigen::Quaterniond last_orientation_d_target(this->orientation_d_);
    Eigen::Quaterniond orientation_d;
    orientation_d.coeffs() << msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
        msg.pose.orientation.w;

    if (last_orientation_d_target.coeffs().dot(this->orientation_d_.coeffs()) < 0.0)
    {
      this->orientation_d_.coeffs() << -this->orientation_d_.coeffs();
    }
    this->setReferencePose(position_d, orientation_d);
  }


  void CartesianImpedanceControllerRos::wrenchCommandCb(const geometry_msgs::msg::WrenchStamped &msg){
    Eigen::Matrix<double, 6, 1> F;
    F << msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y,
        msg.wrench.torque.z;

    if (!msg.header.frame_id.empty() && msg.header.frame_id != this->root_frame_)
    {
      if (!transformWrench(&F, msg.header.frame_id, this->root_frame_))
      {
        RCLCPP_ERROR(this->get_node()->get_logger(), "Could not transform wrench. Not applying it.");
        return;
      }
    }
    else if (msg.header.frame_id.empty())
    {
      if (!transformWrench(&F, this->wrench_ee_frame_, this->root_frame_))
      {
        RCLCPP_ERROR(this->get_node()->get_logger(), "Could not transform wrench. Not applying it.");
        return;
      }
    }

    this->applyWrench(F);
  }

  void CartesianImpedanceControllerRos::updateParams(){
    
    // Check for parameter changes
    if (param_listener_->is_old(params_)) {
      RCLCPP_INFO(this->get_node()->get_logger(), "Importing updated parameters.");
      params_ = param_listener_->get_params();

      // Update stiffness from parameters
      geometry_msgs::msg::Wrench cart_stiffness;
      cart_stiffness.force.x = params_.stiffness.force.x;
      cart_stiffness.force.y = params_.stiffness.force.y;
      cart_stiffness.force.z = params_.stiffness.force.z;
      cart_stiffness.torque.x = params_.stiffness.torque.x;
      cart_stiffness.torque.y = params_.stiffness.torque.y;
      cart_stiffness.torque.z = params_.stiffness.torque.z;
      double null_stiffness = params_.stiffness.nullspace;
      setStiffness(cart_stiffness, null_stiffness);

      // Update damping from parameters
      geometry_msgs::msg::Wrench cart_damping;
      cart_damping.force.x = params_.damping.force.x;
      cart_damping.force.y = params_.damping.force.y;
      cart_damping.force.z = params_.damping.force.z;
      cart_damping.torque.x = params_.damping.torque.x;
      cart_damping.torque.y = params_.damping.torque.y;
      cart_damping.torque.z = params_.damping.torque.z;
      double null_damping = params_.damping.nullspace;
      this->setDampingFactors(cart_damping, null_damping);

      integral_stiffness_coefficent_ = params_.integral_control.stiffness_coefficent;
      integral_leak_rate_ = params_.integral_control.leak_rate;
    }
  }

  // Declares this controller
  PLUGINLIB_EXPORT_CLASS(rcm_controllers::CartesianImpedanceControllerRos,
                         controller_interface::ControllerInterface);
                         
} // namespace cartesian_impedance_controller
