#ifndef CARTESIAN_IMPEDANCE_TRAJECTORY_CONTROLLER_ROS_
#define CARTESIAN_IMPEDANCE_TRAJECTORY_CONTROLLER_ROS_

#include <mutex>
#include <chrono>
#include <functional> 
#include <memory>
#include <string>
#include <vector>

#include "Eigen/Dense"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_action/server.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "realtime_tools/realtime_server_goal_handle.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "control_msgs/srv/query_trajectory_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "rcm_cartesian_impedance_controller/cartesian_impedance_controller.h"
#include "rcm_cartesian_impedance_controller/rbdyn_wrapper.h"

#include "rcm_cartesian_impedance_controller/msg/controller_config.hpp"
#include "rcm_cartesian_impedance_controller/msg/controller_state.hpp"

#include <rcm_cartesian_impedance_controller/cartesian_impedance_controller_ros_parameters.hpp>

using namespace std::chrono_literals;

namespace rcm_controllers
{
  /*! \brief The ROS2 control implementation of the Cartesian impedance controller
  * 
  * It utilizes a list of joint names and the URDF description to control these joints.
  * 
  * The ROS2 implementation is partially modeled after the default 
  * JointTrajectoryController implementation in ROS2 Controllers. This is the main
  * inspiration for the 'public' and 'protected' sections of the class, while
  * the 'private' section contains methods from the original ROS1 implemenation.
  * The inherited CartesianImpedanceController class is where the actual control
  * law is applied, and remains mostly unchanged.
  *
  */
  class CartesianImpedanceControllerRos
      : public controller_interface::ControllerInterface, public CartesianImpedanceController
  {
  public:

    CartesianImpedanceControllerRos();

    /**
     * @brief Configures ros2_control command interfaces
     */
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    /**
     * @brief Configures ros2_control state interfaces
     */
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    /**
     * @brief Updates controller state when active
     */
    controller_interface::return_type update(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;
    
    /**
     * @brief Initializes the controller
     */
    controller_interface::CallbackReturn on_init() override;
    
    /**
     * @brief Configures the controller based on initial parameters
     */
    controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State & previous_state) override;
    
    /**
     * @brief Prepares the controller to take control of the robot
     */  
    controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State & previous_state) override;

    /**
     * @brief Prepares the controller to release control of the robot
     */
    controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State & previous_state) override;
  
  protected:

    using FollowJTrajAction = control_msgs::action::FollowJointTrajectory;
    using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<FollowJTrajAction>;
    using RealtimeGoalHandlePtr = std::shared_ptr<RealtimeGoalHandle>;
    using RealtimeGoalHandleBuffer = realtime_tools::RealtimeBuffer<RealtimeGoalHandlePtr>;

    using ControllerStateMsg = control_msgs::msg::JointTrajectoryControllerState;
    using StatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;
    using StatePublisherPtr = std::unique_ptr<StatePublisher>;

    // callback for topic interface
    void topic_callback(const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> msg);

    // callbacks for action_server_
    rclcpp_action::GoalResponse goal_received_callback(
      const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const FollowJTrajAction::Goal> goal);

    rclcpp_action::CancelResponse goal_cancelled_callback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle);

    void goal_accepted_callback(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle);

    void preempt_active_goal();
    
    bool contains_interface_type(
      const std::vector<std::string> & interface_type_list, const std::string & interface_type);

    // To reduce number of variables and to make the code shorter the interfaces are ordered in types
    // as the following constants
    const std::vector<std::string> allowed_interface_types_ = {
      hardware_interface::HW_IF_POSITION,
      hardware_interface::HW_IF_VELOCITY,
      hardware_interface::HW_IF_ACCELERATION,
      hardware_interface::HW_IF_EFFORT,
    };

    // Preallocate variables used in the realtime update() function
    trajectory_msgs::msg::JointTrajectoryPoint state_current_;
    trajectory_msgs::msg::JointTrajectoryPoint command_current_;
    trajectory_msgs::msg::JointTrajectoryPoint state_desired_;
    trajectory_msgs::msg::JointTrajectoryPoint state_error_;

    // Degrees of freedom
    size_t dof_;

    // Storing command joint names for interfaces
    std::vector<std::string> command_joint_names_;

    // Parameters from ROS for joint_trajectory_controller
    std::shared_ptr<ParamListener> param_listener_;
    Params params_;

    trajectory_msgs::msg::JointTrajectoryPoint last_commanded_state_;

    // The interfaces are defined as the types in 'allowed_interface_types_' member.
    // For convenience, for each type the interfaces are ordered so that i-th position
    // matches i-th index in joint_names_
    template <typename T>
    using InterfaceReferences = std::vector<std::vector<std::reference_wrapper<T>>>;

    InterfaceReferences<hardware_interface::LoanedCommandInterface> joint_command_interface_;
    InterfaceReferences<hardware_interface::LoanedStateInterface> joint_state_interface_;

    bool has_position_state_interface_ = false;
    bool has_velocity_state_interface_ = false;
    bool has_acceleration_state_interface_ = false;
    bool has_position_command_interface_ = false;
    bool has_velocity_command_interface_ = false;
    bool has_acceleration_command_interface_ = false;
    bool has_effort_command_interface_ = false;

    std::vector<bool> joints_angle_wraparound_;
    // reserved storage for result of the command when closed loop pid adapter is used
    std::vector<double> tmp_command_;

    // Timeout to consider commands old
    double cmd_timeout_;
    // True if holding position or repeating last trajectory point in case of success
    realtime_tools::RealtimeBuffer<bool> rt_is_holding_;
    // TODO(karsten1987): eventually activate and deactivate subscriber directly when its supported
    bool subscriber_is_active_ = false;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_command_subscriber_ =
      nullptr;

    rclcpp::Publisher<ControllerStateMsg>::SharedPtr publisher_legacy_;
    StatePublisherPtr state_publisher_legacy_;
    rclcpp::Publisher<ControllerStateMsg>::SharedPtr publisher_;
    StatePublisherPtr state_publisher_;

    rclcpp::Duration state_publisher_period_ = rclcpp::Duration(20ms);
    rclcpp::Time last_state_publish_time_;

    rclcpp_action::Server<FollowJTrajAction>::SharedPtr action_server_;
    RealtimeGoalHandleBuffer rt_active_goal_;  ///< Currently active action goal, if any.
    realtime_tools::RealtimeBuffer<bool> rt_has_pending_goal_;  ///< Is there a pending action goal?
    rclcpp::TimerBase::SharedPtr goal_handle_timer_;
    rclcpp::Duration action_monitor_period_ = rclcpp::Duration(50ms);
  

  private:

    /*! \brief Initializes the controller
    *
    * - Reads ROS parameters
    * \return             True on success, false on failure
    */

    bool initParams();
    
    /*! \brief Starts the controller
    *
    * Updates the states and sets the desired pose and nullspace configuration to the current state.
    */
    void starting();

    /*! \brief Initializes messaging
    *
    * Initializes realtime publishers and the subscribers.
    * \param[in] nh Nodehandle
    * \return True on success, false on failure.
    */
    bool initMessaging();

    /*! \brief Initializes RBDyn
    *
    * Reads the robot URDF and initializes RBDyn.
    * \param[in] nh Nodehandle
    * \return True on success, false on failure.
    */
    bool initRBDyn();

    /*! \brief Initializes trajectory handling
    *
    * Subscribes to joint trajectory topic and starts the trajectory action server.
    * \param[in] nh Nodehandle
    * \return Always true.
    */
    bool initTrajectories();

    /*! \brief Get forward kinematics solution.
    *
    * Calls RBDyn to get the forward kinematics solution.
    * \param[in]  q            Joint position vector
    * \param[out] position     End-effector position
    * \param[out] orientation  End-effector orientation
    * \return Always true.
    */
    bool getFk(const Eigen::VectorXd &q, Eigen::Vector3d *position, Eigen::Quaterniond *rotation) const;

    /*! \brief Get Jacobian from RBDyn
    *
    * Gets the Jacobian for given joint positions and joint velocities.
    * \param[in]  q         Joint position vector        
    * \param[in]  dq        Joint velocity vector
    * \param[out] jacobian  Calculated Jacobian
    * \return True on success, false on failure.
    */
    bool getJacobian(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, Eigen::MatrixXd *jacobian);

    /*! \brief Updates the state based on the joint handles.
    *
    * Gets latest joint positions, velocities and efforts and updates the forward kinematics as well as the Jacobian. 
    */
    void updateState();

    /*! \brief Sets damping for Cartesian space and nullspace.
    *
    * Long
    * \param[in] cart_damping   Cartesian damping [0,1]
    * \param[in] nullspace      Nullspace damping [0,1]
    */
    void setDampingFactors(const geometry_msgs::msg::Wrench &cart_damping, double nullspace);

    /*! \brief Sets Cartesian and nullspace stiffness
    *
    * Sets Cartesian and nullspace stiffness. Allows to set if automatic damping should be applied.
    * \param[in] cart_stiffness Cartesian stiffness
    * \param[in] nullspace      Nullspace stiffness
    * \param[in] auto_damping   Apply automatic damping 
    */
    void setStiffness(const geometry_msgs::msg::Wrench &cart_stiffness, double nullspace, bool auto_damping = true);
    
    /*! \brief Message callback for Cartesian stiffness.
    *
    * Calls setStiffness function.
    * @sa setStiffness
    * \param[in] msg Received message
    */
    void cartesianDampingFactorCb(const geometry_msgs::msg::WrenchStamped &msg);

    /*! \brief Message callback for Cartesian stiffness.
    *
    * Calls setStiffness function.
    * @sa setStiffness
    * \param[in] msg Received message
    */
    void cartesianStiffnessCb(const geometry_msgs::msg::WrenchStamped &msg);

    /*! \brief Message callback for the whole controller configuration.
    *
    * Sets stiffness, damping and nullspace.
    * @sa setDampingFactors, setStiffness
    * \param[in] msg Received message
    */
    void controllerConfigCb(const rcm_cartesian_impedance_controller::msg::ControllerConfig &msg);

    /*! \brief Message callback for a Cartesian reference pose.
    *
    * Accepts new reference poses in the root frame - ignores them otherwise.
    * Sets the reference target pose.
    * @sa setReferencePose.
    * \param[in] msg Received message
    */
    void referencePoseCb(const geometry_msgs::msg::PoseStamped &msg);

    /*! \brief Message callback for Cartesian wrench messages.
    *
    * If the wrench is not given in end-effector frame, it will be transformed in the root frame. Once when a new wrench message arrives.
    * Sets the wrench using the base library.
    * @sa applyWrench.
    * \param[in] msg Received message
    */
    void wrenchCommandCb(const geometry_msgs::msg::WrenchStamped &msg);

    /*! \brief Transforms the wrench in a target frame.
    *
    * Takes a vector with the wrench and transforms it to a given coordinate frame. E.g. from_frame= "world" , to_frame = "bh_link_ee"

    * @sa wrenchCommandCb
    * \param[in] cartesian_wrench Vector with the Cartesian wrench
    * \param[in] from_frame       Source frame
    * \param[in] to_frame         Target frame
    * \return True on success, false on failure.
    */
    bool transformWrench(Eigen::Matrix<double, 6, 1> *cartesian_wrench, const std::string &from_frame, const std::string &to_frame) const;

    /*! \brief Verbose printing; publishes ROS messages and tf frames.
     *
     * Always publishes commanded torques.
     * Optional: request publishes tf frames for end-effector forward kinematics and the reference pose.
     * Optional: verbose printing
     * Optional: publishes state messages
     */
    void publishMsgsAndTf();

    /*! \brief Starts the trajectory.
    *
    * All in one execution for single point trajectories, for servoing etc.
    */
    void trajExecute(const trajectory_msgs::msg::JointTrajectory &trajectory);

    /*! \brief Starts the trajectory.
    *
    * Resets the trajectory member variables. 
    */
    void trajStart(const trajectory_msgs::msg::JointTrajectory &trajectory);

    /*! \brief Updates the trajectory.
    *
    * Called periodically from the update function if a trajectory is running.
    * A trajectory is run by going through it point by point, calculating forward kinematics and applying
    * the joint configuration to the nullspace control.
    */
    void trajUpdate();

    void robotDescriptionCb(const std_msgs::msg::String &msg);

    void updateParams();
    
    rbdyn_wrapper rbdyn_wrapper_;   //!< Wrapper for RBDyn library for kinematics 
    std::string end_effector_;      //!< End-effector link name

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_robot_description_; //!< URDF subscription
    std::mutex robot_description_mutex_; //!< Mutex to safely handle updating from subscription
    std::string robot_description_buffer_; //!< URDF of the robot, updated by the subscription
    std::string robot_description_; //!< URDF of the robot
    std::string root_frame_;        //!< Base frame obtained from URDF

    Eigen::VectorXd tau_m_;         //!< Measured joint torques
    
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_damping_factors_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_cartesian_stiffness_;    //!< Cartesian stiffness subscriber
    rclcpp::Subscription<rcm_cartesian_impedance_controller::msg::ControllerConfig>::SharedPtr sub_controller_config_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_reference_pose_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_wrench_command_;


    std::string wrench_ee_frame_;           //!< Frame for the application of the commanded wrench 

    // Hard limits. They are enforced on input.
    const double trans_stf_min_{0};     //!< Minimum translational stiffness
    const double trans_stf_max_{1500};  //!< Maximum translational stiffness
    const double rot_stf_min_{0};       //!< Minimum rotational stiffness
    const double rot_stf_max_{100};     //!< Maximum rotational stiffness
    const double ns_min_{0};            //!< Minimum nullspace stiffness
    const double ns_max_{100};          //!< Maximum nullspace stiffness
    const double dmp_factor_min_{0.001};       //!< Minimum damping factor
    const double dmp_factor_max_{2.0};         //!< Maximum damping factor

    // The Jacobian of RBDyn comes with orientation in the first three lines. Needs to be interchanged.
    const Eigen::VectorXi perm_indices_ =
      (Eigen::Matrix<int, 6, 1>() << 3, 4, 5, 0, 1, 2).finished(); //!< Permutation indices to switch position and orientation
    const Eigen::PermutationMatrix<Eigen::Dynamic, 6> jacobian_perm_ =
      Eigen::PermutationMatrix<Eigen::Dynamic, 6>(perm_indices_); //!< Permutation matrix to switch position and orientation entries

    trajectory_msgs::msg::JointTrajectory trajectory_;
    rclcpp::Time traj_start_;          //!< Time the current trajectory is started 
    rclcpp::Duration traj_duration_ = rclcpp::Duration(1s);   //!< Duration of the current trajectory
    unsigned int traj_index_{0};    //!< Index of the current trajectory point
    bool traj_running_{false};      //!< True when running a trajectory

    // Extra output
    bool verbose_print_{false};       //!< Verbose printing enabled
    bool verbose_state_{false};       //!< Verbose state messages enabled
    bool verbose_tf_{false};          //!< Verbose tf pubishing enabled

    tf2::Transform tf_br_transform_;   //!< tf transform for publishing
    tf2::Vector3 tf_pos_;              //!< tf position for publishing
    tf2::Quaternion tf_rot_;           //!< tf orientation for publishing
    rclcpp::Time tf_last_time_; //!< Last published tf message

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_torques_;  //!< Publisher for commanded torques
    rclcpp::Publisher<rcm_cartesian_impedance_controller::msg::ControllerState>::SharedPtr pub_state_;  //!< Publisher for controller state

  };

} // namespace cartesian_impedance_controller

#endif