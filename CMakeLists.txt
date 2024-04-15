cmake_minimum_required(VERSION 3.8)
project(moveit_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  trajectory_msgs
  geometry_msgs
  std_srvs
  urdf
  moveit_ros_planning_interface
  moveit_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

set(HEADER_FILES include/moveit_planning/moveit_planning.hpp)

add_library(moveit_planning_lib src/moveit_planning.cpp ${HEADER_FILES}) 
ament_target_dependencies(moveit_planning_lib ${dependencies})
ament_export_targets(moveit_planning_lib HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/moveit_planning
  DESTINATION include
)

add_executable(multiple_example src/examples/multiple_example.cpp src/moveit_planning.cpp ${HEADER_FILES})
ament_target_dependencies(multiple_example ${dependencies})
  
install(TARGETS
multiple_example
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS moveit_planning_lib
  EXPORT moveit_planning_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
