#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <tf2/LinearMath/Quaternion.h>

using moveit::planning_interface::MoveGroupInterface;
using std::placeholders::_1;
using namespace std::chrono_literals;

typedef struct
{
    int num_attempts = 1;
    int replan_attempts = 0;
    double planning_time = 1.0;
    double constrained_planning_time = 5.0;
} internal_planner_settings;

using Kinematics_info = std::pair<const moveit::core::JointModelGroup *, std::string>; // needed by IKFast

class moveit_planning
{
public:
    moveit_planning(rclcpp::Node::SharedPtr node);
    moveit_planning(rclcpp::Node::SharedPtr node, const std::string& planning_group, const internal_planner_settings &settings);
    ~moveit_planning();

    // planning
    bool plan_cartesian_trajectory(const std::vector<geometry_msgs::msg::Pose> &waypoints, trajectory_msgs::msg::JointTrajectory &trajectory, const double &interpolation_step = 0.01, const double &eef_step = 0.001, const bool &collision_check = false);
    bool plan_cartesian_trajectory(const std::vector<geometry_msgs::msg::Pose> &waypoints, trajectory_msgs::msg::JointTrajectory &trajectory, const std::string &object, const double &interpolation_step = 0.01, const double &eef_step = 0.001, const double &penetration = 0.03);
    bool plan_trajectory(const std::vector<geometry_msgs::msg::Pose> &target_poses, trajectory_msgs::msg::JointTrajectory &trajectory, const std::vector<double> &orientation_tolerances = std::vector<double>());
    bool plan_trajectory(const std::vector<double> &target_joint_state, trajectory_msgs::msg::JointTrajectory &trajectory, const std::vector<double> &orientation_tolerances = std::vector<double>());
 
    // planning_scene
    bool update_scene(const moveit_msgs::msg::PlanningScene &scene);
    Eigen::Affine3d get_object_pose(const std::string &object_id);
    Eigen::Affine3d get_attached_object_pose(const std::string &object_id);
    void set_robot_state(const std::vector<double> &robot_configuration);
    void display_trajectory(const trajectory_msgs::msg::JointTrajectory &trajectory, const int &sleep_ms_between_states = 50);

    /*
    object shape type:
    -   BOX = 1
    -   SPHERE = 2
    -   CYLINDER = 3
    -   CONE = 4
    */
    void create_object( const int& object_shape_type, 
                        const std::string &name, 
                        const std::vector<double>& size,
                        const Eigen::Affine3d& pose,
                        const std::vector<double>& color,
                        const std::string& ref_link = "world");
    void remove_object(const std::string &name);
    moveit_msgs::msg::PlanningScene attach_object(const std::string &object_id, const std::string &link);
    moveit_msgs::msg::PlanningScene detach_object(const std::string &object_id);

    // kinematics
    void inverse_kinematics(const geometry_msgs::msg::Pose pose, std::vector<std::vector<double>> &solutions, const std::vector<double> robot_configuration_to_compare = std::vector<double>());
    void inverse_kinematics(const Eigen::Affine3d pose, std::vector<std::vector<double>> &solutions, const std::vector<double> robot_configuration_to_compare = std::vector<double>());
    void forward_kinematics(const std::vector<double> &state, Eigen::Affine3d &pose, const std::string &link = "base_link_gripper");

    // utils
    bool check_state_validity(const std::vector<double> &state, std::vector<std::pair<std::string, std::string>> &contact_pairs);
    trajectory_msgs::msg::JointTrajectory extract_trajectory_point(const trajectory_msgs::msg::JointTrajectory &trajectory, const int &point_index);
    std::vector<double> get_joint_state_ordered(const sensor_msgs::msg::JointState &joint_state);
    bool parametrize_trajectory(trajectory_msgs::msg::JointTrajectory &trajectory);
    void print_attached_objects();
    Eigen::MatrixXd get_jacobian(const std::vector<double> &configuration_reference, const bool &use_quaternion_form = false);
    geometry_msgs::msg::Pose get_current_pose();
    std::vector<double> get_current_configuration();
    std::vector<std::string> get_joint_names();
    Eigen::Affine3d get_base_link_pose_in_world_ref(const Eigen::Affine3d& pose);

private:
    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterfacePtr move_group_interface_;
    planning_scene_monitor::PlanningSceneMonitorPtr psm_;
    planning_scene::PlanningScenePtr planning_scene_;
    planning_scene::PlanningScenePtr utils_planning_scene_;
    Kinematics_info kinematics_info_;
    internal_planner_settings settings_;
    moveit::core::JointModelGroup *joint_model_group_;
};