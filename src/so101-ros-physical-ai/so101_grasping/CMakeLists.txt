cmake_minimum_required(VERSION 3.8)
project(so101_grasping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(so101_kinematics_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(so101_simple_pick src/so101_simple_pick.cpp)
ament_target_dependencies(
  so101_simple_pick
  rclcpp
  rclcpp_action
  control_msgs
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  shape_msgs
  tf2
)

add_executable(so101_visual_grasp src/so101_visual_grasp.cpp)
ament_target_dependencies(
  so101_visual_grasp
  rclcpp
  rclcpp_action
  control_msgs
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  shape_msgs
  so101_kinematics_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS so101_simple_pick
  so101_visual_grasp
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
