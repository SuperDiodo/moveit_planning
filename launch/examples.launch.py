from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    moveit_config_pkg_name = "simple_ur10e"
    moveit_config = MoveItConfigsBuilder(moveit_config_pkg_name).to_moveit_configs()


    general_params = {
        "example_id": 2,
    }

    example_1_params = {
        "position": [0.,0.,0.],
        "angles": [0.,0.,0.],
    }

    example_2_params = {
        "robot_configuration": [0.,0.,0.,0.,0.,0.],
    }
    
    node = Node(
        package="moveit_planning",
        executable="multiple_example",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            general_params,
            example_1_params,
            example_2_params
        ],
    )

    return LaunchDescription([node])