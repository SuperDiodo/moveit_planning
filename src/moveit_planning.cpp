#include <moveit_planning/moveit_planning.hpp>

moveit_planning::moveit_planning(rclcpp::Node::SharedPtr node) : node_(node)
{
    std::string planning_group = node_->declare_parameter<std::string>("planning_group", "arm");
    move_group_interface_ = std::make_shared<MoveGroupInterface>(node, planning_group);
    psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_, "robot_description");
    psm_->requestPlanningSceneState("/get_planning_scene");
    planning_scene_ = psm_->getPlanningScene();
    utils_planning_scene_ = psm_->getPlanningScene();
    kinematics_info_ = std::make_pair(move_group_interface_->getRobotModel()->getJointModelGroup(planning_group), planning_group);
    move_group_interface_->setReplanAttempts(0);
    move_group_interface_->setNumPlanningAttempts(settings_.num_attempts);
    settings_.num_attempts = node_->declare_parameter<int>("num_attempts", 5);
    settings_.replan_attempts = node_->declare_parameter<int>("replan_attempts", 0);
    settings_.planning_time = node->declare_parameter<double>("planning_time", 1.0);
    settings_.constrained_planning_time = node->declare_parameter<double>("constrained_planning_time", 5.0);
}

moveit_planning::moveit_planning(rclcpp::Node::SharedPtr node, const std::string &planning_group, const internal_planner_settings &settings) : node_(node)
{
    move_group_interface_ = std::make_shared<MoveGroupInterface>(node, planning_group);
    psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_, "robot_description");
    psm_->requestPlanningSceneState("/get_planning_scene");
    planning_scene_ = psm_->getPlanningScene();
    utils_planning_scene_ = psm_->getPlanningScene();
    kinematics_info_ = std::make_pair(move_group_interface_->getRobotModel()->getJointModelGroup(planning_group), planning_group);
    move_group_interface_->setReplanAttempts(settings.replan_attempts);
    move_group_interface_->setNumPlanningAttempts(settings.num_attempts);
    settings_ = settings;
}

moveit_planning::~moveit_planning()
{

}

Eigen::Affine3d moveit_planning::get_base_link_pose_in_world_ref(const Eigen::Affine3d &pose)
{
    auto transform = planning_scene_->getFrameTransform(kinematics_info_.first->getSolverInstance()->getBaseFrame());
    return transform * pose;
}

void moveit_planning::set_robot_state(const std::vector<double> &robot_configuration)
{
    auto joint_states_pub = node_->create_publisher<sensor_msgs::msg::JointState>("/fake_joint_states", RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    sensor_msgs::msg::JointState msg;
    msg.header.frame_id = "";
    msg.name = kinematics_info_.first->getJointModelNames();
    msg.header.stamp = node_->get_clock()->now();
    msg.position = robot_configuration;
    joint_states_pub->publish(msg);
    std::cout << "setting robot state to: ";
    for (auto v : robot_configuration)
        std::cout << v << " ";
    std::cout << std::endl;
}

void moveit_planning::display_trajectory(const trajectory_msgs::msg::JointTrajectory &trajectory, const int &sleep_ms_between_states)
{
    auto joint_states_pub = node_->create_publisher<sensor_msgs::msg::JointState>("/fake_joint_states", RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    sensor_msgs::msg::JointState msg;
    msg.header.frame_id = "";
    msg.name = trajectory.joint_names;

    for (auto point : trajectory.points)
    {
        msg.header.stamp = node_->get_clock()->now();
        msg.position = point.positions;
        joint_states_pub->publish(msg);
        rclcpp::sleep_for(std::chrono::milliseconds(sleep_ms_between_states));
    }
}

bool moveit_planning::plan_cartesian_trajectory(const std::vector<geometry_msgs::msg::Pose> &waypoints, trajectory_msgs::msg::JointTrajectory &trajectory, const std::string &object, const double &interpolation_step, const double &eef_step, const double &penetration)
{
    moveit_msgs::msg::RobotTrajectory trajectory_;
    psm_->requestPlanningSceneState("/get_planning_scene");
    planning_scene_ = psm_->getPlanningScene();

    RCLCPP_INFO(node_->get_logger(), "Getting current state");
    move_group_interface_->setStartState(planning_scene_->getCurrentState());

    std::vector<geometry_msgs::msg::Pose> interpolated_waypoints;
    for (size_t i = 0; i < waypoints.size() - 1; i++)
    {

        interpolated_waypoints.push_back(waypoints[i]);

        double dx = waypoints[i + 1].position.x - waypoints[i].position.x;
        double dy = waypoints[i + 1].position.y - waypoints[i].position.y;
        double dz = waypoints[i + 1].position.z - waypoints[i].position.z;
        double dist = sqrt(dx * dx + dy * dy + dz * dz);

        const int num_interpolation_to_perform = dist / interpolation_step;

        for (double t = 0; t < 1; t += 1.0 / (1.0 + (double)num_interpolation_to_perform))
        {
            Eigen::Quaterniond quat_a;
            quat_a.x() = waypoints[i].orientation.x;
            quat_a.y() = waypoints[i].orientation.y;
            quat_a.z() = waypoints[i].orientation.z;
            quat_a.w() = waypoints[i].orientation.w;

            Eigen::Quaterniond quat_b;
            quat_b.x() = waypoints[i + 1].orientation.x;
            quat_b.y() = waypoints[i + 1].orientation.y;
            quat_b.z() = waypoints[i + 1].orientation.z;
            quat_b.w() = waypoints[i + 1].orientation.w;

            Eigen::Quaterniond quat_interpolated = quat_a.slerp(t, quat_b);
            geometry_msgs::msg::Pose inteprolated_pose;
            inteprolated_pose.position.x = t * waypoints[i + 1].position.x + (1 - t) * waypoints[i].position.x;
            inteprolated_pose.position.y = t * waypoints[i + 1].position.y + (1 - t) * waypoints[i].position.y;
            inteprolated_pose.position.z = t * waypoints[i + 1].position.z + (1 - t) * waypoints[i].position.z;
            inteprolated_pose.orientation.x = quat_interpolated.x();
            inteprolated_pose.orientation.y = quat_interpolated.y();
            inteprolated_pose.orientation.z = quat_interpolated.z();
            inteprolated_pose.orientation.w = quat_interpolated.w();

            interpolated_waypoints.push_back(inteprolated_pose);
        }
    }

    interpolated_waypoints.push_back(waypoints.back());

    move_group_interface_->computeCartesianPath(interpolated_waypoints, eef_step, 0.0, trajectory_, false);

    for (size_t i = 0; i < trajectory_.joint_trajectory.points.size(); i++)
    {
        auto state = trajectory_.joint_trajectory.points[i].positions;
        std::vector<std::pair<std::string, std::string>> contact_pairs;
        if (check_state_validity(state, contact_pairs))
            continue;
        for (auto contact_pair : contact_pairs)
        {
            if (contact_pair.first != object && contact_pair.second != object)
            {
                return false;
            }
        }

        std::cout << "Touched object at point " << i << " of " << trajectory_.joint_trajectory.points.size() << std::endl;
        Eigen::Affine3d fk_pose, ideal_pose;
        forward_kinematics(trajectory_.joint_trajectory.points[i].positions, fk_pose);

        int points_after_penetration = 0;
        double total_penetration = 0.0;

        for (size_t j = i + 1; j < trajectory_.joint_trajectory.points.size(); j++)
        {
            Eigen::Affine3d fk_pose_pressing;
            forward_kinematics(trajectory_.joint_trajectory.points[j].positions, fk_pose_pressing);
            double dist = (fk_pose.translation() - fk_pose_pressing.translation()).norm();
            std::cout << "At point " << j << " the eef moved for " << dist << " m" << std::endl;
            total_penetration += dist;
            if (total_penetration >= penetration)
                break;
            points_after_penetration++;
            fk_pose.translation() = fk_pose_pressing.translation();
        }

        trajectory_.joint_trajectory.points.erase(trajectory_.joint_trajectory.points.begin() + i + points_after_penetration, trajectory_.joint_trajectory.points.end());
        parametrize_trajectory(trajectory_.joint_trajectory);
        trajectory = trajectory_.joint_trajectory;
        return true;
    }

    return false;
}

bool moveit_planning::plan_cartesian_trajectory(const std::vector<geometry_msgs::msg::Pose> &waypoints, trajectory_msgs::msg::JointTrajectory &trajectory, const double &interpolation_step, const double &eef_step, const bool &collision_check)
{
    moveit_msgs::msg::RobotTrajectory trajectory_;
    psm_->requestPlanningSceneState("/get_planning_scene");
    planning_scene_ = psm_->getPlanningScene();

    std::vector<geometry_msgs::msg::Pose> interpolated_waypoints;
    for (size_t i = 0; i < waypoints.size() - 1; i++)
    {

        interpolated_waypoints.push_back(waypoints[i]);

        double dx = waypoints[i + 1].position.x - waypoints[i].position.x;
        double dy = waypoints[i + 1].position.y - waypoints[i].position.y;
        double dz = waypoints[i + 1].position.z - waypoints[i].position.z;
        double dist = sqrt(dx * dx + dy * dy + dz * dz);

        const int num_interpolation_to_perform = dist / interpolation_step;

        std::cout << "interpolated waypoints positions: " << std::endl;
        for (double t = 0; t < 1; t += 1.0 / (1.0 + (double)num_interpolation_to_perform))
        {
            Eigen::Quaterniond quat_a;
            quat_a.x() = waypoints[i].orientation.x;
            quat_a.y() = waypoints[i].orientation.y;
            quat_a.z() = waypoints[i].orientation.z;
            quat_a.w() = waypoints[i].orientation.w;

            Eigen::Quaterniond quat_b;
            quat_b.x() = waypoints[i + 1].orientation.x;
            quat_b.y() = waypoints[i + 1].orientation.y;
            quat_b.z() = waypoints[i + 1].orientation.z;
            quat_b.w() = waypoints[i + 1].orientation.w;

            Eigen::Quaterniond quat_interpolated = quat_a.slerp(t, quat_b);
            geometry_msgs::msg::Pose inteprolated_pose;
            inteprolated_pose.position.x = t * waypoints[i + 1].position.x + (1 - t) * waypoints[i].position.x;
            inteprolated_pose.position.y = t * waypoints[i + 1].position.y + (1 - t) * waypoints[i].position.y;
            inteprolated_pose.position.z = t * waypoints[i + 1].position.z + (1 - t) * waypoints[i].position.z;
            inteprolated_pose.orientation.x = quat_interpolated.x();
            inteprolated_pose.orientation.y = quat_interpolated.y();
            inteprolated_pose.orientation.z = quat_interpolated.z();
            inteprolated_pose.orientation.w = quat_interpolated.w();
            std::cout << inteprolated_pose.position.x << ", " << inteprolated_pose.position.y << ", " << inteprolated_pose.position.z << std::endl;
            interpolated_waypoints.push_back(inteprolated_pose);
        }
    }

    std::cout << waypoints.back().position.x << ", " << waypoints.back().position.y << ", " << waypoints.back().position.z << std::endl;
    interpolated_waypoints.push_back(waypoints.back());

    RCLCPP_INFO(node_->get_logger(), "Getting current state");
    move_group_interface_->setStartState(planning_scene_->getCurrentState());
    auto percentage_planned_trajectory = move_group_interface_->computeCartesianPath(interpolated_waypoints, eef_step, 0.0, trajectory_, collision_check);

    std::cout << "planned " << percentage_planned_trajectory << " of cartesian traj" << std::endl;

    if (percentage_planned_trajectory == 1.0)
    {
        trajectory = trajectory_.joint_trajectory;
        return true;
    }
    return false;
}

bool moveit_planning::plan_trajectory(const std::vector<geometry_msgs::msg::Pose> &target_poses, trajectory_msgs::msg::JointTrajectory &trajectory, const std::vector<double> &orientation_tolerances)
{

    if (orientation_tolerances.size() > 0)
    {
        std::cout << "Setting orientation constraint" << std::endl;
        moveit_msgs::msg::OrientationConstraint orientation_constraint;
        orientation_constraint.header.frame_id = move_group_interface_->getPoseReferenceFrame();
        orientation_constraint.link_name = move_group_interface_->getEndEffectorLink();
        orientation_constraint.orientation = move_group_interface_->getCurrentPose(orientation_constraint.link_name).pose.orientation;
        orientation_constraint.absolute_x_axis_tolerance = orientation_tolerances[0];
        orientation_constraint.absolute_y_axis_tolerance = orientation_tolerances[1];
        orientation_constraint.absolute_z_axis_tolerance = orientation_tolerances[2];
        orientation_constraint.weight = 1.0;
        moveit_msgs::msg::Constraints orientation_constraints;
        orientation_constraints.orientation_constraints.emplace_back(orientation_constraint);
        move_group_interface_->setPathConstraints(orientation_constraints);
        move_group_interface_->setPlanningTime(settings_.constrained_planning_time);
    }
    else
    {
        move_group_interface_->clearPathConstraints();
        move_group_interface_->setPlanningTime(settings_.planning_time);
    }

    psm_->requestPlanningSceneState("/get_planning_scene");
    planning_scene_ = psm_->getPlanningScene();

    RCLCPP_INFO(node_->get_logger(), "Getting current state");
    move_group_interface_->setStartState(planning_scene_->getCurrentState());
    move_group_interface_->setPoseTargets(target_poses);
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = move_group_interface_->plan(msg) == moveit::core::MoveItErrorCode::SUCCESS;
    move_group_interface_->clearPathConstraints();

    if (ok)
    {
        trajectory = msg.trajectory_.joint_trajectory;
        return true;
    }
    return false;
}

bool moveit_planning::plan_trajectory(const std::vector<double> &target_joint_state, trajectory_msgs::msg::JointTrajectory &trajectory, const std::vector<double> &orientation_tolerances)
{

    if (orientation_tolerances.size() > 0)
    {
        std::cout << "Setting orientation constraint" << std::endl;
        moveit_msgs::msg::OrientationConstraint orientation_constraint;
        orientation_constraint.header.frame_id = move_group_interface_->getPoseReferenceFrame();
        orientation_constraint.link_name = move_group_interface_->getEndEffectorLink();
        orientation_constraint.orientation = move_group_interface_->getCurrentPose(orientation_constraint.link_name).pose.orientation;
        orientation_constraint.absolute_x_axis_tolerance = orientation_tolerances[0];
        orientation_constraint.absolute_y_axis_tolerance = orientation_tolerances[1];
        orientation_constraint.absolute_z_axis_tolerance = orientation_tolerances[2];
        orientation_constraint.weight = 1.0;
        moveit_msgs::msg::Constraints orientation_constraints;
        orientation_constraints.orientation_constraints.emplace_back(orientation_constraint);
        move_group_interface_->setPathConstraints(orientation_constraints);
        move_group_interface_->setPlanningTime(settings_.constrained_planning_time);
    }
    else
    {
        move_group_interface_->clearPathConstraints();
        move_group_interface_->setPlanningTime(settings_.planning_time);
    }

    psm_->requestPlanningSceneState("/get_planning_scene");
    planning_scene_ = psm_->getPlanningScene();

    RCLCPP_INFO(node_->get_logger(), "Getting current state");
    move_group_interface_->setStartState(planning_scene_->getCurrentState());

    move_group_interface_->setJointValueTarget(target_joint_state);

    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = move_group_interface_->plan(msg) == moveit::core::MoveItErrorCode::SUCCESS;
    move_group_interface_->clearPathConstraints();
    if (ok)
    {
        trajectory = msg.trajectory_.joint_trajectory;
        auto final_traj_point = extract_trajectory_point(trajectory, trajectory.points.size() - 1);

        bool approximate_solution = false;
        std::cout << "Plan found with ik solution, it deviates from goal of: ";
        for (size_t i = 0; i < final_traj_point.joint_names.size(); i++)
        {
            double err = abs(target_joint_state[i] - final_traj_point.points[0].positions[i]);
            std::cout << err << " ";
            if (err > 0.001)
                approximate_solution = true;
        }
        std::cout << std::endl;

        if (approximate_solution)
            std::cout << "it is an approximate solution, ignoring it.." << std::endl;

        return !approximate_solution;
    }
    return false;
}

void moveit_planning::print_attached_objects()
{
    std::vector<moveit_msgs::msg::AttachedCollisionObject> a_objects;
    planning_scene_->getAttachedCollisionObjectMsgs(a_objects);
    for (auto a_o : a_objects)
        std::cout << a_o.object.id << " attached to " << a_o.link_name << std::endl;
}

void moveit_planning::create_object(
    const int &object_shape_type,
    const std::string &name,
    const std::vector<double> &size,
    const Eigen::Affine3d &pose,
    const std::vector<double> &color,
    const std::string &ref_link)
{

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    moveit_msgs::msg::CollisionObject collision_object_msgs;
    {
        collision_object_msgs.header.frame_id = ref_link;
        collision_object_msgs.id = name;
        collision_object_msgs.primitives.resize(1);
        collision_object_msgs.primitives[0].type = object_shape_type;
        collision_object_msgs.primitives[0].dimensions.resize(3);
        collision_object_msgs.primitives[0].dimensions[0] = size[0];
        collision_object_msgs.primitives[0].dimensions[1] = size[1];
        collision_object_msgs.primitives[0].dimensions[2] = size[2];
        collision_object_msgs.primitive_poses.resize(1);
        collision_object_msgs.primitive_poses[0].position.x = pose.translation().x();
        collision_object_msgs.primitive_poses[0].position.y = pose.translation().y();
        collision_object_msgs.primitive_poses[0].position.z = pose.translation().z();
        auto quat = (Eigen::Quaterniond)pose.linear();
        collision_object_msgs.primitive_poses[0].orientation.x = quat.x();
        collision_object_msgs.primitive_poses[0].orientation.y = quat.y();
        collision_object_msgs.primitive_poses[0].orientation.z = quat.z();
        collision_object_msgs.primitive_poses[0].orientation.w = quat.w();
        collision_object_msgs.operation = collision_object_msgs.ADD;
    }

    moveit_msgs::msg::ObjectColor color_msgs;
    {
        color_msgs.color.r = color[0];
        color_msgs.color.g = color[1];
        color_msgs.color.b = color[2];
        color_msgs.color.a = color[3];
    }

    planning_scene_interface.applyCollisionObject(collision_object_msgs, color_msgs.color);
}

void moveit_planning::remove_object(const std::string &name)
{
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_msgs::msg::CollisionObject remove_object;
    remove_object.id = name;
    remove_object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    planning_scene_interface.applyCollisionObject(remove_object);
}

moveit_msgs::msg::PlanningScene moveit_planning::attach_object(const std::string &object_id, const std::string &link)
{
    move_group_interface_->attachObject(object_id, link);
    return moveit_msgs::msg::PlanningScene();
}

moveit_msgs::msg::PlanningScene moveit_planning::detach_object(const std::string &object_id)
{
    move_group_interface_->detachObject(object_id);
    return moveit_msgs::msg::PlanningScene();
}

geometry_msgs::msg::Pose moveit_planning::get_current_pose() { return move_group_interface_->getCurrentPose().pose; };

Eigen::Affine3d moveit_planning::get_object_pose(const std::string &object_id)
{
    psm_->requestPlanningSceneState("/get_planning_scene");
    planning_scene_ = psm_->getPlanningScene();
    moveit_msgs::msg::CollisionObject c_object;
    if (!planning_scene_->getCollisionObjectMsg(c_object, object_id))
        std::cout << "collision object " << object_id << " does not exists" << std::endl;
    Eigen::Affine3d object_pose;
    object_pose.translation() << c_object.pose.position.x, c_object.pose.position.y, c_object.pose.position.z;
    object_pose.linear() = Eigen::Quaterniond(c_object.pose.orientation.w, c_object.pose.orientation.x, c_object.pose.orientation.y, c_object.pose.orientation.z).toRotationMatrix();
    return object_pose;
};

Eigen::Affine3d moveit_planning::get_attached_object_pose(const std::string &object_id)
{

    psm_->requestPlanningSceneState("/get_planning_scene");
    planning_scene_ = psm_->getPlanningScene();
    moveit_msgs::msg::AttachedCollisionObject ac_object;
    if (!planning_scene_->getAttachedCollisionObjectMsg(ac_object, object_id))
        std::cout << "attached collision object " << object_id << " does not exists" << std::endl;
    Eigen::Affine3d object_pose;
    auto c_object = ac_object.object;
    object_pose.translation() << c_object.pose.position.x, c_object.pose.position.y, c_object.pose.position.z;
    object_pose.linear() = Eigen::Quaterniond(c_object.pose.orientation.w, c_object.pose.orientation.x, c_object.pose.orientation.y, c_object.pose.orientation.z).toRotationMatrix();
    return object_pose;
};

trajectory_msgs::msg::JointTrajectory moveit_planning::extract_trajectory_point(const trajectory_msgs::msg::JointTrajectory &trajectory, const int &point_index)
{
    trajectory_msgs::msg::JointTrajectory temp_trajectory;
    temp_trajectory.header = trajectory.header;
    temp_trajectory.joint_names = trajectory.joint_names;
    temp_trajectory.points.push_back(trajectory.points[point_index]);
    return temp_trajectory;
}

bool moveit_planning::update_scene(const moveit_msgs::msg::PlanningScene &scene)
{

    planning_scene_->setPlanningSceneMsg(scene);

    using srv_type = moveit_msgs::srv::ApplyPlanningScene;
    rclcpp::Client<srv_type>::SharedPtr client = node_->create_client<srv_type>("/apply_planning_scene");

    using srv_request = srv_type::Request;
    auto request = std::make_shared<srv_request>();
    request->scene = scene;

    while (!client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted");
            return false;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
    }

    auto response_future = client->async_send_request(request).future.share();
    std::chrono::seconds wait_time(1);
    std::future_status fs = response_future.wait_for(wait_time);
    if (fs == std::future_status::timeout)
    {
        RCLCPP_ERROR(node_->get_logger(), "Service timed out.");
    }
    else
    {
        std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene_Response> planning_response;
        planning_response = response_future.get();
        if (planning_response->success)
        {
            RCLCPP_INFO(node_->get_logger(), "Service successfully updated scene.");
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Service failed to update scene.");
        }
    }
    return true;
}

std::vector<double> moveit_planning::get_joint_state_ordered(const sensor_msgs::msg::JointState &joint_state)
{

    std::unordered_map<std::string, double> mapped_joint_values;
    auto joint_names = kinematics_info_.first->getVariableNames();
    for (auto jj : joint_names)
    {
        std::cout << jj << std::endl;
        mapped_joint_values[jj] = 0.0;
    }

    for (size_t j = 0; j < joint_state.name.size(); j++)
        mapped_joint_values[joint_state.name[j]] = joint_state.position[j];

    std::vector<double> ordered_joint_values;
    for (auto m_j_v : mapped_joint_values)
    {
        ordered_joint_values.push_back(m_j_v.second);
    }

    std::reverse(ordered_joint_values.begin(), ordered_joint_values.end());
    return ordered_joint_values;
}

bool moveit_planning::parametrize_trajectory(trajectory_msgs::msg::JointTrajectory &trajectory)
{
    trajectory_processing::TimeOptimalTrajectoryGeneration traj_opt_gen;
    robot_trajectory::RobotTrajectory robot_trajectory(move_group_interface_->getRobotModel());
    robot_trajectory.setGroupName("arm");
    auto msg = robot_trajectory.setRobotTrajectoryMsg(*move_group_interface_->getCurrentState().get(), trajectory);
    if (!traj_opt_gen.computeTimeStamps(msg))
    {
        return false;
    }

    moveit_msgs::msg::RobotTrajectory result;
    msg.getRobotTrajectoryMsg(result);
    trajectory = result.joint_trajectory;
    return true;
}

void moveit_planning::inverse_kinematics(const geometry_msgs::msg::Pose pose, std::vector<std::vector<double>> &solutions, const std::vector<double> robot_configuration_to_compare)
{
    Eigen::Affine3d eigen_pose;
    eigen_pose.translation().x() = pose.position.x;
    eigen_pose.translation().y() = pose.position.y;
    eigen_pose.translation().z() = pose.position.z;
    Eigen::Quaterniond quat;
    quat.x() = pose.orientation.x;
    quat.y() = pose.orientation.y;
    quat.z() = pose.orientation.z;
    quat.w() = pose.orientation.w;
    eigen_pose.linear() = quat.toRotationMatrix();
    inverse_kinematics(eigen_pose, solutions, robot_configuration_to_compare);
}

double euclideanDistance(const std::vector<double> &point1, const std::vector<double> &point2)
{
    if (point1.size() != point2.size())
    {
        // Handle error: Points should have the same dimension
        return -1.0;
    }

    double distance = 0.0;
    for (size_t i = 0; i < point1.size(); ++i)
    {
        distance += std::pow(point1[i] - point2[i], 2);
    }

    return std::sqrt(distance);
}

bool compareDistances(const std::vector<double> &point1, const std::vector<double> &point2, const std::vector<double> &reference)
{
    double distance1 = euclideanDistance(point1, reference);
    double distance2 = euclideanDistance(point2, reference);
    return distance1 < distance2;
}

void moveit_planning::inverse_kinematics(const Eigen::Affine3d pose, std::vector<std::vector<double>> &solutions, const std::vector<double> robot_configuration_to_compare)
{

    solutions.clear();

    std::vector<double> seed(kinematics_info_.first->getJointModelNames().size(), 0.0);
    kinematics::KinematicsResult result;
    kinematics::KinematicsQueryOptions options;
    moveit_msgs::msg::MoveItErrorCodes error_code;

    auto transform = planning_scene_->getFrameTransform(kinematics_info_.first->getSolverInstance()->getBaseFrame());
    auto transformed_pose = transform.inverse() * pose;

    std::cout << "pose to convert:\n"
              << transformed_pose.matrix() << std::endl;

    geometry_msgs::msg::Pose pose_to_convert;
    pose_to_convert.position.x = transformed_pose.translation().x();
    pose_to_convert.position.y = transformed_pose.translation().y();
    pose_to_convert.position.z = transformed_pose.translation().z();
    auto quat_transformed_pose = ((Eigen::Quaterniond)transformed_pose.linear());
    pose_to_convert.orientation.x = quat_transformed_pose.x();
    pose_to_convert.orientation.y = quat_transformed_pose.y();
    pose_to_convert.orientation.z = quat_transformed_pose.z();
    pose_to_convert.orientation.w = quat_transformed_pose.w();

    std::cout << "eef: " << kinematics_info_.first->getEndEffectorName() << std::endl;

    kinematics_info_.first->getSolverInstance()->getPositionIK({pose_to_convert}, seed, solutions, result, options);
    std::cout << solutions.size() << " ik solutions found: " << std::endl;

    // if comparison robot configuration is provided, order by euclidean distance
    if (robot_configuration_to_compare.size() == kinematics_info_.first->getJointModelNames().size())
    {
        std::sort(solutions.begin(), solutions.end(), [&](const auto &a, const auto &b)
                  { return compareDistances(a, b, robot_configuration_to_compare); });
    }

    for (auto sol : solutions)
    {
        std::cout << "[ ";
        for (auto jv : sol)
            std::cout << jv << " ";
        std::cout << "]" << std::endl;
    }
}

void moveit_planning::forward_kinematics(const std::vector<double> &state, Eigen::Affine3d &pose, const std::string &link)
{
    moveit::core::RobotState &current_state = planning_scene_->getCurrentStateNonConst();
    current_state.setJointGroupPositions(kinematics_info_.first, state);
    const Eigen::Isometry3d &end_effector_state = current_state.getGlobalLinkTransform(link);
    pose.translation() = end_effector_state.translation();
    pose.linear() = end_effector_state.linear();
}

bool moveit_planning::check_state_validity(const std::vector<double> &state, std::vector<std::pair<std::string, std::string>> &contact_pairs)
{
    psm_->requestPlanningSceneState("/get_planning_scene");
    planning_scene_ = psm_->getPlanningScene();
    moveit::core::RobotState &current_state = planning_scene_->getCurrentStateNonConst();
    current_state.setJointGroupPositions(kinematics_info_.first, state);
    collision_detection::CollisionRequest collision_request;
    collision_request.contacts = true;
    collision_request.max_contacts = 100;
    collision_request.max_contacts_per_pair = 1;
    collision_detection::CollisionResult collision_result;
    planning_scene_->checkCollision(collision_request, collision_result);

    /* Check is state is colliding */
    if (collision_result.collision)
    {
        for (auto c : collision_result.contacts)
        {
            contact_pairs.push_back(std::make_pair(c.first.first, c.first.second));
        }

        return false;
    }

    return true;
}

Eigen::MatrixXd moveit_planning::get_jacobian(const std::vector<double> &configuration_reference, const bool &use_quaternion)
{
    moveit::core::RobotState &temp_state = utils_planning_scene_->getCurrentStateNonConst();
    temp_state.setJointGroupPositions(kinematics_info_.first, configuration_reference);

    if (!use_quaternion)
        return temp_state.getJacobian(kinematics_info_.first);
    else
    {
        Eigen::MatrixXd J;
        temp_state.getJacobian(kinematics_info_.first, kinematics_info_.first->getLinkModels().back(), Eigen::Vector3d(0., 0., 0.), J, use_quaternion);
        return J;
    }
}

std::vector<double> moveit_planning::get_current_configuration()
{
    return move_group_interface_->getCurrentJointValues();
}

std::vector<std::string> moveit_planning::get_joint_names()
{
    return kinematics_info_.first->getJointModelNames();
}
