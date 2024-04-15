# C++ library for ROS2 MoveIt! planning framework

C++ library which includes utility functions for using ROS2 MoveIt! planning framework.

This project is currently under development, with ongoing updates and enhancements planned for the future. If you have any thoughts or feedback about the software, feel free to contact us at alessio.saccuti@unipr.it.

**A more detailed documentation will be available soon!**

---

## `moveit_planning` capabilities

- Plan cartesian trajectories
- Plan collision-free trajectories with RRTConnect
- Custom planning scenes handling (check robot conf. validity, attach/detach objects, create/remove objects)
- Inverse and forward kinematics with the kinematic plugin defined in the moveit configuration package
- Display of simulated trajectories with RViz

---

## Setup MoveIt! and `moveit_planning`

Install MoveIt!:

```bash
sudo apt install ros-humble-moveit
sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc
```

Setup Moveit Planning library:

```bash
git clone https://github.com/SuperDiodo/moveit_planning.git
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select moveit_planning
```

Create MoveIt! configuration package

- Create it from scratch using a robot URDF and [moveit setup assistant](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html#moveit-setup-assistant).

- Use an already existing MoveIt! conf. package. An example can be found in [ur_ros_rtde](https://github.com/SuperDiodo/ur_ros_rtde) repository in which an UR10e MoveIt! conf. package is provided.

Build the configuration package with:
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select your_moveit_config_pkg
```


## Integrate MoveIt! and `moveit_planning` in your ROS2 project


1. Link `moveit_planning` in your ROS2 package `CMakeLists.txt`:

    ```bash
    # CMakeLists.txt

    ...your stuff

    find_package(moveit_planning REQUIRED)
    find_package(moveit_ros_planning_interface REQUIRED)
    find_package(moveit_msgs REQUIRED)

    ...your stuff

    set(moveit_dependencies
    moveit_ros_planning_interface
    moveit_msgs
    )

    ...your stuff

    # example of executable called my_executable, generated using source1.cpp and source2.cpp (use yours!)
    add_executable(my_executable source1.cpp source2.cpp ...)
    ament_target_dependencies(my_executable ${moveit_dependencies})
    target_link_libraries(my_executable moveit_planning::moveit_planning_lib)
    
    ...your stuff
    ```

2. Create a launch file for launching your code and add all the required lines for integrating MoveIt! functionalities:

    ```python
    from launch import LaunchDescription
    from launch_ros.actions import Node
    from moveit_configs_utils import MoveItConfigsBuilder

    def generate_launch_description():

        moveit_config_pkg_name = "your_moveit_config_pkg_name"
        moveit_config = MoveItConfigsBuilder(moveit_config_pkg_name).to_moveit_configs()

        params = {
            # put your params here!
        }
        

        node = Node(
            package="your_ros2_pkg_name",
            executable="my_executable",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                params,
            ],
        )

        return LaunchDescription([node])
    ```

3. Create an instance of Moveit Planning in your code:

    ```cpp
    int main(int argc, char *argv[])
    {
        rclcpp::init(argc, argv);

        auto moveit_interface_node = std::make_shared<rclcpp::Node>("moveit_interface_node");

        // use a default constructor
        auto planning_interface = moveit_planning(moveit_interface_node);

        // OR pass custom planning group name and planner setting
        std::string planning_group = "arm";
        internal_planner_settings settings;
        settings_.planning_time = 5.0;
        planning_interface = moveit_planning(moveit_interface_node, planning_group, settings);

        // starts a background spinner for internal functionalities
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(moveit_interface_node);
        std::thread([&executor]()
                    { executor.spin(); })
            .detach();

        // ... do what you usually do from here ... //

        // ...

    }
    ```

## Usage examples

Examples of usage of `moveit_planning` are provided: trajectory planning towards a target pose (**example 0**), validation of a robot configuration (**example 1**) and creation of collision objects (**example 2**).


We will use [ur_ros_rtde](https://github.com/SuperDiodo/ur_ros_rtde) simulation mode, which allows to automatically launch the required launch files from a MoveIt! configuration package.

#### Launch `ur_ros_rdte` in simulation mode:

1. configure `ur_ros_rtde` as shown in the setup guide
2. configure `ur_ros_rtde/launch/robot_state_receiver.launch.py` setting:
   - `moveit_config_pkg = "simple_ur10e_moveit_config`
   - `robot_description_package = "simple_ur10e_secription`
   - `launch_moveit = True`
   - `simulation_only = True`

3. launch `ur_ros_rtde` robot state receiver:
    ```bash
    # type in a new terminal
    ros2 launch ur_ros_rtde robot_state_receiver.launch.py
    ```

#### Test `moveit_planning` with the examples provided:

1. configure `moveit_planning/launch/examples.launch.py` setting: 
   - `moveit_config_pkg_name = "simple_ur10e"`   
   - `"example_id": 0` or `"example_id": 1` or `"example_id": 2` based on which example should be launched
   - set target `position` and `angles` if example 0 is selected
   - set `robot_configuration` to validate if example 1 is selected

2. launch the example:
    ```bash
    # type in a new terminal
    ros2 launch moveit_planning examples.launch.py
    ```