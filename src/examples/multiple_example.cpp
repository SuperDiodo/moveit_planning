#include <moveit_planning/moveit_planning.hpp>

#define EXAMPLE_PLAN_TRAJECTORY 0
#define EXAMPLE_ROBOT_CONF_VALIDATION 1
#define EXAMPLE_ADD_OBJECTS 2

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto moveit_interface_node = std::make_shared<rclcpp::Node>("moveit_interface_node");
    auto planning_interface = moveit_planning(moveit_interface_node);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(moveit_interface_node);
    std::thread([&executor]()
                { executor.spin(); })
        .detach();

    auto example_id = moveit_interface_node->declare_parameter<int>("example_id", 0);

    switch (example_id)
    {
    case EXAMPLE_PLAN_TRAJECTORY:
    {
        std::vector<double> position = moveit_interface_node->declare_parameter<std::vector<double>>("position", {0., 0., 0.});
        std::vector<double> angles = moveit_interface_node->declare_parameter<std::vector<double>>("angles", {0., 0., 0.});
        Eigen::AngleAxisd rollAngle(angles[0], Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(angles[1], Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(angles[2], Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond quaternion = yawAngle * pitchAngle * rollAngle;
        Eigen::Matrix3d rotationMatrix = quaternion.matrix();
        Eigen::Translation3d translation(position[0],position[1],position[2]);
        Eigen::Affine3d target_pose = translation * rotationMatrix;
        // If IKFast is used retrieves all the ik solutions and sort them based on the actual robot conf.
        std::vector<std::vector<double>> ik_solutions;
        auto actual_robot_conf = planning_interface.get_current_configuration();
        planning_interface.inverse_kinematics(target_pose, ik_solutions, actual_robot_conf);
        bool success = false;
        // One after the other use an ik solution for motion planning, exit if a trajectory is found
        for (auto ik_sol : ik_solutions)
        {
            trajectory_msgs::msg::JointTrajectory planned_trajectory;
            planning_interface.set_robot_state(actual_robot_conf);
            success = planning_interface.plan_trajectory(ik_sol, planned_trajectory);
            if(success) break;
        }
        std::cout << "Planning " << (success ? "Success" : "Failed") << std::endl;
        break;
    }

    case EXAMPLE_ROBOT_CONF_VALIDATION:
    {
        std::vector<double> state = moveit_interface_node->declare_parameter<std::vector<double>>("robot_configuration", {0., 0., 0., 0., 0., 0.});
        std::vector<std::pair<std::string, std::string>> contact_pairs;
        if (planning_interface.check_state_validity(state, contact_pairs) == false)
        {
            std::cout << "Invalid state, contacts found:" << std::endl;
            for (auto contact_pair : contact_pairs)
                std::cout << contact_pair.first << "~" << contact_pair.second << std::endl;
        }
        else
            std::cout << "Valid state" << std::endl;
        break;
    }

    case EXAMPLE_ADD_OBJECTS:
    {
        Eigen::Affine3d obj_pose = Eigen::Affine3d::Identity();
        std::vector<double> obj_dimension = {0.1, 0.1, 0.1};
        std::vector<double> color_red = {1.0, 0.0, 0.0, 1.0};
        std::vector<double> color_green = {0.0, 1.0, 0.0, 1.0};
        std::vector<double> color_blue = {0.0, 0.0, 1.0, 1.0};
        std::vector<double> color_black = {0.0, 0.0, 0.0, 1.0};

        // create a box
        {
            obj_pose.translation().y() += 0.5;
            planning_interface.create_object(1, "box", obj_dimension, obj_pose, color_red);
        }
        // create a sphere
        {
            obj_pose.translation().y() -= 1.0;
            planning_interface.create_object(2, "sphere", obj_dimension, obj_pose, color_green);
        }
        // create a cylinder
        {
            obj_pose.translation().y() += 0.5;
            obj_pose.translation().x() -= 0.5;
            planning_interface.create_object(3, "cylinder", obj_dimension, obj_pose, color_blue);
        }

        // create a cone
        {
             obj_pose.translation().x() += 1.0;
            planning_interface.create_object(4, "cone", obj_dimension, obj_pose, color_black);
        }
        break;
    }

    default:
        std::cout << "Example " << example_id << " does not exist!" << std::endl;
        break;
    }

    rclcpp::shutdown();
    return 0;
}