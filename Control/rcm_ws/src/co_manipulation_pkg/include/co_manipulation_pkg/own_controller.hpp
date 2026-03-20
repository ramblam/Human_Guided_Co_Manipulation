#ifndef CO_MANIPULATION_PKG_OWN_CONTROLLER_HPP
#define CO_MANIPULATION_PKG_OWN_CONTROLLER_HPP

#include <atomic>
#include <set>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rcm_clients/cartesian_movement_client.hpp"
#include "rcm_clients/control_end_effector_client.hpp"
#include "rcm_clients/get_pose_client.hpp"
#include "rcm_clients/joint_movement_client.hpp"
#include "rcm_clients/get_joint_states_client.hpp"
#include "rcm_clients/rcm_manager_client.hpp"
#include "rcm_clients/pose_movement_client.hpp"
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include "std_srvs/srv/trigger.hpp"

class OwnController{
    public:
        OwnController(std::shared_ptr<rclcpp::Node>& node);

        void set_force(double force);

        bool pick(const geometry_msgs::msg::Pose target_pose, double distance, bool advanced_mode = false);

        bool place(const geometry_msgs::msg::Pose starting_pose, double distance, bool advanced_mode = false);

        bool control_end_effector(const std::vector<std::string>& io_name, const std::vector<long>& io_state, double width, double force, double angle);

        bool update_current_pose(geometry_msgs::msg::Pose& pose);

        bool move_to_pose(const geometry_msgs::msg::Pose& pose, double speed, double timeout);

        bool move_to_joints(std::vector<double> joint_positions);

        bool move_home();

        bool activate_wrist_following_ref();

        bool deactivate_wrist_following_ref();

        bool activate_wrist_following_servo();

        bool deactivate_wrist_following_servo();

        bool allow_compliance_3d();

        bool disallow_compliance_3d();

        bool allow_compliance_6d();

        bool disallow_compliance_6d();

        bool allow_compliance_plane();

        bool disallow_compliance_plane();

        bool allow_compliance_also(std::string direction);

        bool disallow_compliance_also(std::string direction);

        bool allow_compliance_only(std::string direction);

        bool disallow_compliance_only(std::string direction);

        bool resist_more(std::string direction);

        bool resist_less(std::string direction);

        bool set_reference_frame_to_current_ee_pose();

        bool set_reference_frame(geometry_msgs::msg::PoseStamped& reference_pose);

        void follow_wrist_ref_frame();

        void follow_wrist_servo();

        bool restart_controller(const std::string &controller_to_start);
        
        bool switch_to_compliance_controller();
        
        bool switch_to_default_controller();

        bool open_gripper();

        bool close_gripper();

        void pick_ply(std::function<void(bool)> result_cb);

        bool end_hand_guiding();

        bool move_close_to_pick();

        bool approach_placing_point();

    protected:
        bool approach(const geometry_msgs::msg::Pose target_pose, double distance, bool advanced_mode);

        bool retreat(const geometry_msgs::msg::Pose starting_pose, const geometry_msgs::msg::Pose target_pose, double distance, bool advanced_mode);

        geometry_msgs::msg::Pose translate_in_target_frame(const geometry_msgs::msg::Pose target_pose, double distance);

        geometry_msgs::msg::Pose compensate_ee_orientation(const geometry_msgs::msg::Pose target_pose);

        void r_wrist_pose_cb(const geometry_msgs::msg::PoseStamped& r_wrist_pose);

        void start_safe_tracking();

        std::shared_ptr<rclcpp::Node> node_;
        std::vector<double> default_ee_orientation_;
        std::vector<std::string> io_name_;
        std::vector<long int> io_state_;
        double width_ = 0.0;
        std::atomic<double> force_ = 0.5;
        double angle_ = 0.0;

        double force_step_size_ = 100.0;
        double torque_step_size_ = 10.0;
        double default_force_ = 300.0;
        double default_torque_ = 30.0;
        double current_speed_ = 0.0;
        
        rcm_clients::ControlEndEffectorClient control_end_effector_client_;
        rcm_clients::GetPoseClient get_pose_client_;
        rcm_clients::CartesianMovementClient cartesian_movement_client_;
        rcm_clients::JointMovementClient joint_movement_client_;
        rcm_clients::GetJointsClient get_joints_client_;
        rcm_clients::RCMManagerClient rcm_manager_client_;
        rcm_clients::PoseMovementClient pose_movement_client_;

        rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr servo_cmd_type_changer_;
        std::shared_ptr<rclcpp::AsyncParametersClient> remote_client_;

        geometry_msgs::msg::WrenchStamped current_compliance_msg_;
        std::set<std::string> current_compliance_directions_ = {};
        std::set<std::string> all_compliance_directions_ = {"x", "y", "z", "rot_x", "rot_y", "rot_z"};

        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr stiffness_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ref_frame_publisher_;
        bool updating_ref_pose_ = false;
        double ref_pose_error_x_ = 0.0;
        double ref_pose_error_y_ = 0.0;
        double ref_pose_error_z_ = 0.0;

        int wrist_following_round_ref_ = 0;
        bool wrist_following_ref_activated_ = false;
        double big_ply_length_ = 0.95;
        double big_ply_width_ = 0.50;
        double small_ply_length_ = 0.53;

        int wrist_following_round_servo_ = 0;
        bool wrist_following_servo_activated_ = false;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr r_wrist_pose_sub_;
        geometry_msgs::msg::Pose current_pose_compliance_controller_;
        geometry_msgs::msg::Pose current_r_wrist_pose_;
        geometry_msgs::msg::Pose prev_r_wrist_pose_;
        geometry_msgs::msg::Pose starting_pose_wrist_following_servo_;
        geometry_msgs::msg::Pose starting_pose_wrist_following_ref_;

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr servo_pose_publisher_;
        rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr controller_switcher_client_;
        std::string current_controller_;
        double default_stiffness_ = 0.5;
        std::vector<double> current_stiffnesses_ = {default_stiffness_, default_stiffness_, default_stiffness_, default_stiffness_, default_stiffness_, default_stiffness_};
        double stiffness_step_size_;
        bool compliance_controller_activated_;
        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr gravity_comp_wrench_publisher_;    
        std::vector<double> home_joint_values_;
        bool operation_active_ = false;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr static_grasp_client_;
};

#endif