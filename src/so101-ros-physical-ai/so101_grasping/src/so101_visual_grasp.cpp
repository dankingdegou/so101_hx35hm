#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/parallel_gripper_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <so101_kinematics_msgs/srv/go_to_pose.hpp>
#include <so101_kinematics_msgs/srv/go_to_joints.hpp>
#include <std_msgs/msg/float64.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <deque>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <thread>
#include <algorithm>
#include <vector>

using GripperCommand = control_msgs::action::ParallelGripperCommand;
using GoToPose = so101_kinematics_msgs::srv::GoToPose;
using GoToJoints = so101_kinematics_msgs::srv::GoToJoints;

static double median_of(std::vector<double> values)
{
  if (values.empty()) return 0.0;
  const auto mid = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
  std::nth_element(values.begin(), mid, values.end());
  if ((values.size() % 2U) == 1U) return *mid;
  const auto lower_mid = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2 - 1);
  std::nth_element(values.begin(), lower_mid, values.end());
  return 0.5 * (*mid + *lower_mid);
}

static bool ensure_robot_model_params_from_move_group(
  const rclcpp::Node::SharedPtr& node,
  const std::string& move_group_node)
{
  if (!node) return false;

  try {
    const auto urdf = node->get_parameter("robot_description").as_string();
    const auto srdf = node->get_parameter("robot_description_semantic").as_string();
    if (!urdf.empty() && !srdf.empty()) return true;
  } catch (...) {
  }

  auto client = std::make_shared<rclcpp::AsyncParametersClient>(node, move_group_node);
  if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(
      node->get_logger(),
      "move_group parameters service not available at '%s'. Start MoveIt first (move_group).",
      move_group_node.c_str());
    return false;
  }

  const std::vector<std::string> keys = {"robot_description", "robot_description_semantic"};
  auto fut = client->get_parameters(keys);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  const auto rc = exec.spin_until_future_complete(fut, std::chrono::seconds(5));
  exec.remove_node(node);

  if (rc != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Timed out fetching robot_description from '%s'.", move_group_node.c_str());
    return false;
  }

  const auto params = fut.get();
  if (params.size() != keys.size()) {
    RCLCPP_ERROR(node->get_logger(), "Unexpected parameter list size from move_group.");
    return false;
  }

  const auto urdf2 = params[0].as_string();
  const auto srdf2 = params[1].as_string();
  if (urdf2.empty() || srdf2.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Fetched empty robot model params from '%s' (urdf=%zu bytes, srdf=%zu bytes).",
      move_group_node.c_str(), urdf2.size(), srdf2.size());
    return false;
  }

  node->set_parameter(rclcpp::Parameter("robot_description", urdf2));
  node->set_parameter(rclcpp::Parameter("robot_description_semantic", srdf2));
  RCLCPP_INFO(
    node->get_logger(),
    "Loaded robot model params from '%s' (urdf=%zu bytes, srdf=%zu bytes).",
    move_group_node.c_str(), urdf2.size(), srdf2.size());

  auto copy_ns = [&](const std::string& prefix) {
    auto list_fut = client->list_parameters({prefix}, 10);
    rclcpp::executors::SingleThreadedExecutor e;
    e.add_node(node);
    const auto lrc = e.spin_until_future_complete(list_fut, std::chrono::seconds(5));
    e.remove_node(node);
    if (lrc != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN(node->get_logger(), "Timed out listing parameters under '%s' from '%s'.",
                  prefix.c_str(), move_group_node.c_str());
      return;
    }
    const auto result = list_fut.get();
    if (result.names.empty()) return;

    auto get_fut = client->get_parameters(result.names);
    rclcpp::executors::SingleThreadedExecutor e2;
    e2.add_node(node);
    const auto grc = e2.spin_until_future_complete(get_fut, std::chrono::seconds(5));
    e2.remove_node(node);
    if (grc != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN(node->get_logger(), "Timed out getting parameters under '%s' from '%s'.",
                  prefix.c_str(), move_group_node.c_str());
      return;
    }
    for (const auto& p : get_fut.get()) {
      node->set_parameter(p);
    }
    RCLCPP_INFO(node->get_logger(), "Copied %zu parameters under '%s' from '%s'.",
                result.names.size(), prefix.c_str(), move_group_node.c_str());
  };

  copy_ns("robot_description_kinematics");
  copy_ns("robot_description_planning");

  return true;
}

static bool send_gripper_command(
  const rclcpp::Node::SharedPtr& node,
  const std::string& action_name,
  double position,
  double max_effort,
  double timeout_s)
{
  if (!node) return false;
  rclcpp::NodeOptions helper_options;
  helper_options.use_global_arguments(false);
  auto helper = std::make_shared<rclcpp::Node>("so101_visual_grasp_gripper_client", helper_options);
  auto client = rclcpp_action::create_client<GripperCommand>(helper, action_name);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(helper);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  bool server_ready = false;
  while (std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
    if (client->wait_for_action_server(std::chrono::milliseconds(250))) {
      server_ready = true;
      break;
    }
    exec.spin_some();
  }

  if (!server_ready) {
    exec.remove_node(helper);
    RCLCPP_ERROR(node->get_logger(), "Gripper action server not available: %s", action_name.c_str());
    return false;
  }

  GripperCommand::Goal goal;
  goal.command.name = {"gripper"};
  goal.command.position = {position};
  goal.command.velocity = {0.0};
  goal.command.effort = {max_effort};

  auto send_fut = client->async_send_goal(goal);
  if (exec.spin_until_future_complete(send_fut, std::chrono::duration<double>(timeout_s)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    exec.remove_node(helper);
    RCLCPP_ERROR(node->get_logger(), "Timed out sending gripper goal.");
    return false;
  }

  auto goal_handle = send_fut.get();
  if (!goal_handle) {
    exec.remove_node(helper);
    RCLCPP_ERROR(node->get_logger(), "Gripper goal rejected.");
    return false;
  }

  auto res_fut = client->async_get_result(goal_handle);
  if (exec.spin_until_future_complete(res_fut, std::chrono::duration<double>(timeout_s)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    exec.remove_node(helper);
    RCLCPP_ERROR(node->get_logger(), "Timed out waiting for gripper result.");
    return false;
  }
  exec.remove_node(helper);

  const auto wrapped = res_fut.get();
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(node->get_logger(), "Gripper action failed (code=%d).", static_cast<int>(wrapped.code));
    return false;
  }
  return true;
}

static bool send_go_to_pose_request(
  const rclcpp::Node::SharedPtr& node,
  const std::string& service_name,
  const geometry_msgs::msg::PoseStamped& target,
  const std::string& strategy,
  double duration_s,
  double timeout_s)
{
  if (!node) return false;

  rclcpp::NodeOptions helper_options;
  helper_options.use_global_arguments(false);
  auto helper = std::make_shared<rclcpp::Node>("so101_visual_grasp_go_to_pose_client", helper_options);
  auto client = helper->create_client<GoToPose>(service_name);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(helper);

  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  bool server_ready = false;
  while (std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
    if (client->wait_for_service(std::chrono::milliseconds(250))) {
      server_ready = true;
      break;
    }
    exec.spin_some();
  }

  if (!server_ready) {
    exec.remove_node(helper);
    RCLCPP_ERROR(
      node->get_logger(),
      "IK pose service not available: %s",
      service_name.c_str());
    return false;
  }

  auto request = std::make_shared<GoToPose::Request>();
  request->target = target;
  request->strategy = strategy;
  request->duration = duration_s > 0.0 ? duration_s : 0.0;

  auto fut = client->async_send_request(request);
  if (exec.spin_until_future_complete(fut, std::chrono::duration<double>(timeout_s)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    exec.remove_node(helper);
    RCLCPP_ERROR(
      node->get_logger(),
      "Timed out waiting for IK pose service response from %s.",
      service_name.c_str());
    return false;
  }
  exec.remove_node(helper);

  const auto response = fut.get();
  if (!response->success) {
    RCLCPP_ERROR(
      node->get_logger(),
      "IK pose service '%s' failed: %s",
      service_name.c_str(),
      response->message.c_str());
    return false;
  }
  RCLCPP_INFO(
    node->get_logger(),
    "IK pose service '%s' succeeded: %s",
    service_name.c_str(),
    response->message.c_str());
  return true;
}

static bool send_go_to_joints_request(
  const rclcpp::Node::SharedPtr& node,
  const std::string& service_name,
  const std::vector<std::string>& joint_names,
  const std::vector<double>& positions,
  double duration_s,
  double timeout_s)
{
  if (!node) return false;
  if (joint_names.size() != positions.size() || joint_names.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Invalid /go_to_joints request dimensions.");
    return false;
  }

  rclcpp::NodeOptions helper_options;
  helper_options.use_global_arguments(false);
  auto helper = std::make_shared<rclcpp::Node>("so101_visual_grasp_go_to_joints_client", helper_options);
  auto client = helper->create_client<GoToJoints>(service_name);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(helper);

  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  bool server_ready = false;
  while (std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
    if (client->wait_for_service(std::chrono::milliseconds(250))) {
      server_ready = true;
      break;
    }
    exec.spin_some();
  }

  if (!server_ready) {
    exec.remove_node(helper);
    RCLCPP_ERROR(node->get_logger(), "IK joints service not available: %s", service_name.c_str());
    return false;
  }

  auto request = std::make_shared<GoToJoints::Request>();
  request->joint_names = joint_names;
  request->positions = positions;
  request->duration = duration_s > 0.0 ? duration_s : 0.0;

  auto fut = client->async_send_request(request);
  if (exec.spin_until_future_complete(fut, std::chrono::duration<double>(timeout_s)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    exec.remove_node(helper);
    RCLCPP_ERROR(node->get_logger(), "Timed out waiting for IK joints service response from %s.", service_name.c_str());
    return false;
  }
  exec.remove_node(helper);

  const auto response = fut.get();
  if (!response->success) {
    RCLCPP_ERROR(
      node->get_logger(),
      "IK joints service '%s' failed: %s",
      service_name.c_str(),
      response->message.c_str());
    return false;
  }
  RCLCPP_INFO(
    node->get_logger(),
    "IK joints service '%s' succeeded: %s",
    service_name.c_str(),
    response->message.c_str());
  return true;
}

static bool named_pose_to_manipulator_joints(
  const std::string& name,
  std::vector<std::string>& joint_names,
  std::vector<double>& positions)
{
  joint_names = {"elbow_flex", "shoulder_lift", "shoulder_pan", "wrist_flex", "wrist_roll"};
  if (name == "rest") {
    positions = {1.57, -1.57, 0.0, 0.75, 0.0};
    return true;
  }
  if (name == "extended") {
    positions = {-1.57, 1.57, 0.0, 0.0, 0.0};
    return true;
  }
  joint_names.clear();
  positions.clear();
  return false;
}

static bool plan_and_execute_pose(
  moveit::planning_interface::MoveGroupInterface& group,
  const geometry_msgs::msg::PoseStamped& target,
  bool execute,
  bool position_only,
  rclcpp::Logger logger)
{
  group.setStartStateToCurrentState();
  group.clearPoseTargets();
  const auto ee_link = group.getEndEffectorLink();
  if (position_only) {
    group.setPositionTarget(
      target.pose.position.x,
      target.pose.position.y,
      target.pose.position.z,
      ee_link);
  } else {
    group.setPoseTarget(target, ee_link);
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto code = group.plan(plan);
  if (code != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Planning failed (code=%d).", code.val);
    return false;
  }
  if (!execute) {
    RCLCPP_INFO(logger, "Plan OK (execute=false).");
    return true;
  }
  const auto exec_code = group.execute(plan);
  if (exec_code != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Execution failed (code=%d).", exec_code.val);
    return false;
  }
  return true;
}

static bool plan_and_execute_named(
  moveit::planning_interface::MoveGroupInterface& group,
  const std::string& name,
  bool execute,
  rclcpp::Logger logger)
{
  group.setStartStateToCurrentState();
  group.clearPoseTargets();
  if (!group.setNamedTarget(name)) {
    RCLCPP_ERROR(logger, "Failed to set named target '%s'.", name.c_str());
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto code = group.plan(plan);
  if (code != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Planning to named target '%s' failed (code=%d).", name.c_str(), code.val);
    return false;
  }
  if (!execute) {
    RCLCPP_INFO(logger, "Named target '%s' plan OK (execute=false).", name.c_str());
    return true;
  }
  const auto exec_code = group.execute(plan);
  if (exec_code != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Execution to named target '%s' failed (code=%d).", name.c_str(), exec_code.val);
    return false;
  }
  return true;
}

static void log_current_ee_pose(
  moveit::planning_interface::MoveGroupInterface& group,
  rclcpp::Logger logger,
  const std::string& label,
  const std::optional<geometry_msgs::msg::PoseStamped>& target = std::nullopt)
{
  const auto ee_link = group.getEndEffectorLink();
  auto current = group.getCurrentPose(ee_link);
  RCLCPP_INFO(
    logger,
    "%s ee='%s' pose: frame='%s' x=%.3f y=%.3f z=%.3f q=[%.3f, %.3f, %.3f, %.3f]",
    label.c_str(),
    ee_link.c_str(),
    current.header.frame_id.c_str(),
    current.pose.position.x,
    current.pose.position.y,
    current.pose.position.z,
    current.pose.orientation.x,
    current.pose.orientation.y,
    current.pose.orientation.z,
    current.pose.orientation.w);
  if (target.has_value()) {
    const double dx = current.pose.position.x - target->pose.position.x;
    const double dy = current.pose.position.y - target->pose.position.y;
    const double dz = current.pose.position.z - target->pose.position.z;
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    RCLCPP_INFO(
      logger,
      "%s target delta: dx=%+.3f dy=%+.3f dz=%+.3f dist=%.3f",
      label.c_str(),
      dx,
      dy,
      dz,
      dist);
  }
}

static bool apply_table_collision(
  const rclcpp::Node::SharedPtr& node,
  const std::string& frame_id,
  double table_center_x_m,
  double table_center_y_m,
  double table_top_z_m,
  double table_size_x_m,
  double table_size_y_m,
  double table_thickness_m)
{
  if (!node) return false;
  if (table_size_x_m <= 0.0 || table_size_y_m <= 0.0 || table_thickness_m <= 0.0) {
    RCLCPP_WARN(node->get_logger(), "Skip table collision object because one or more dimensions are non-positive.");
    return false;
  }

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit_msgs::msg::CollisionObject table;
  table.header.frame_id = frame_id;
  table.id = "tabletop_guard";

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions = {table_size_x_m, table_size_y_m, table_thickness_m};

  geometry_msgs::msg::Pose table_pose;
  table_pose.orientation.w = 1.0;
  table_pose.position.x = table_center_x_m;
  table_pose.position.y = table_center_y_m;
  table_pose.position.z = table_top_z_m - 0.5 * table_thickness_m;

  table.primitives.push_back(primitive);
  table.primitive_poses.push_back(table_pose);
  table.operation = moveit_msgs::msg::CollisionObject::ADD;

  const bool ok = planning_scene_interface.applyCollisionObjects({table});
  if (ok) {
    RCLCPP_INFO(
      node->get_logger(),
      "Applied tabletop collision guard in frame '%s' (center=[%.3f, %.3f], top_z=%.3f, size=[%.3f, %.3f, %.3f]).",
      frame_id.c_str(),
      table_center_x_m,
      table_center_y_m,
      table_top_z_m,
      table_size_x_m,
      table_size_y_m,
      table_thickness_m);
  } else {
    RCLCPP_WARN(node->get_logger(), "Failed to apply tabletop collision guard to the planning scene.");
  }
  return ok;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "so101_visual_grasp",
    rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true));

  auto declare_if_missing = [&](const std::string& name, const auto& def) {
    if (!node->has_parameter(name)) node->declare_parameter(name, def);
  };

  declare_if_missing("move_group_node", std::string{"/move_group"});
  declare_if_missing("arm_group", std::string{"manipulator"});
  declare_if_missing("ik_tip_link", std::string{"gripper_frame_link"});
  declare_if_missing("target_frame", std::string{"base_link"});
  declare_if_missing("pose_topic", std::string{"/vision/aruco/pose_base"});
  declare_if_missing("execute", true);
  declare_if_missing("enable_arm", true);
  declare_if_missing("enable_gripper", true);
  declare_if_missing("wait_pose_timeout_s", 30.0);
  declare_if_missing("fresh_pose_slack_s", 2.5);
  declare_if_missing("pose_sample_window_s", 0.35);
  declare_if_missing("pose_sample_count", 5);
  declare_if_missing("min_pose_sample_count", 3);
  declare_if_missing("max_pose_spread_m", 0.03);
  declare_if_missing("post_rest_settle_s", 1.0);
  declare_if_missing("pose_transform_timeout_s", 0.1);
  declare_if_missing("pregrasp_offset_m", 0.08);
  declare_if_missing("hover_high_offset_m", 0.0);
  declare_if_missing("pregrasp_x_offset_m", 0.0);
  declare_if_missing("pregrasp_y_offset_m", 0.0);
  declare_if_missing("pregrasp_z_extra_m", 0.0);
  declare_if_missing("use_two_stage_pregrasp", false);
  declare_if_missing("grasp_x_offset_m", 0.0);
  declare_if_missing("grasp_y_offset_m", 0.0);
  declare_if_missing("grasp_z_offset_m", -0.015);
  declare_if_missing("enforce_table_z_limits", true);
  declare_if_missing("tabletop_z_m", 0.0);
  declare_if_missing("use_tabletop_z_topic", true);
  declare_if_missing("tabletop_z_topic", std::string{"/vision/table/top_z"});
  declare_if_missing("wait_tabletop_z_timeout_s", 2.0);
  declare_if_missing("fresh_tabletop_z_slack_s", 1.0);
  declare_if_missing("min_grasp_clearance_m", 0.010);
  declare_if_missing("min_pregrasp_clearance_m", 0.060);
  declare_if_missing("add_table_collision", true);
  declare_if_missing("table_center_x_m", 0.32);
  declare_if_missing("table_center_y_m", 0.0);
  declare_if_missing("table_size_x_m", 0.36);
  declare_if_missing("table_size_y_m", 0.45);
  declare_if_missing("table_thickness_m", 0.04);
  declare_if_missing("position_only", true);
  declare_if_missing("go_to_rest_before_grasp", false);
  declare_if_missing("staging_named_pose", std::string{"rest"});
  declare_if_missing("return_to_named_pose_after_grasp", false);
  declare_if_missing("post_grasp_named_pose", std::string{"rest"});
  declare_if_missing("open_gripper_after_return", false);
  declare_if_missing("post_return_gripper_settle_s", 0.4);
  declare_if_missing("post_grasp_lift_before_return", false);
  declare_if_missing("post_grasp_return_lift_z_m", 0.14);
  declare_if_missing("post_grasp_return_lift_extra_m", 0.04);
  declare_if_missing("ik_post_grasp_lift_strategy", std::string{"cartesian"});
  declare_if_missing("ik_post_grasp_lift_duration_s", 1.2);
  declare_if_missing("ik_post_grasp_lift_settle_s", 0.25);
  declare_if_missing("post_grasp_return_via_rest_hover", false);
  declare_if_missing("post_grasp_rest_hover_x_m", 0.20);
  declare_if_missing("post_grasp_rest_hover_y_m", 0.0);
  declare_if_missing("post_grasp_rest_hover_z_m", 0.14);
  declare_if_missing("ik_post_grasp_rest_hover_strategy", std::string{"cartesian"});
  declare_if_missing("ik_post_grasp_rest_hover_duration_s", 1.4);
  declare_if_missing("ik_post_grasp_rest_hover_settle_s", 0.25);
  declare_if_missing("post_grasp_use_ik_joints", false);
  declare_if_missing("ik_joints_service_name", std::string{"/go_to_joints"});
  declare_if_missing("ik_post_grasp_duration_s", 2.4);
  declare_if_missing("use_marker_orientation", false);
  declare_if_missing("use_rpy", false);
  declare_if_missing("qx", 0.0);
  declare_if_missing("qy", 1.0);
  declare_if_missing("qz", 0.0);
  declare_if_missing("qw", 0.0);
  declare_if_missing("roll", 0.0);
  declare_if_missing("pitch", 0.0);
  declare_if_missing("yaw", 0.0);
  declare_if_missing("planning_time_s", 5.0);
  declare_if_missing("vel_scaling", 0.2);
  declare_if_missing("acc_scaling", 0.2);
  declare_if_missing("use_ik_pose_service", true);
  declare_if_missing("ik_pose_service_name", std::string{"/go_to_pose"});
  declare_if_missing("ik_pose_strategy", std::string{"joint_quintic"});
  declare_if_missing("ik_pregrasp_strategy", std::string{"joint_quintic"});
  declare_if_missing("ik_grasp_strategy", std::string{"cartesian"});
  declare_if_missing("ik_retreat_strategy", std::string{"cartesian"});
  declare_if_missing("ik_pregrasp_duration_s", 1.8);
  declare_if_missing("ik_grasp_duration_s", 1.2);
  declare_if_missing("ik_retreat_duration_s", 1.2);
  declare_if_missing("ik_pose_service_timeout_s", 8.0);
  declare_if_missing("ik_pregrasp_settle_s", 0.35);
  declare_if_missing("ik_grasp_settle_s", 0.25);
  declare_if_missing("ik_retreat_settle_s", 0.25);
  declare_if_missing("gripper_action_name", std::string{"/follower/gripper_controller/gripper_cmd"});
  declare_if_missing("gripper_open_pos", 1.5);
  declare_if_missing("gripper_closed_pos", -0.16);
  declare_if_missing("gripper_max_effort", 0.0);
  declare_if_missing("action_timeout_s", 10.0);
  declare_if_missing("grasp_retry_count", 1);
  declare_if_missing("grasp_retry_delay_s", 0.5);
  declare_if_missing("post_grasp_return_retry_count", 3);
  declare_if_missing("post_grasp_return_retry_delay_s", 1.0);

  const auto move_group_node = node->get_parameter("move_group_node").as_string();
  const auto arm_group = node->get_parameter("arm_group").as_string();
  const auto ik_tip_link = node->get_parameter("ik_tip_link").as_string();
  const auto target_frame = node->get_parameter("target_frame").as_string();
  const auto pose_topic = node->get_parameter("pose_topic").as_string();
  const auto execute = node->get_parameter("execute").as_bool();
  const auto enable_arm = node->get_parameter("enable_arm").as_bool();
  const auto enable_gripper = node->get_parameter("enable_gripper").as_bool();
  const auto wait_pose_timeout_s = node->get_parameter("wait_pose_timeout_s").as_double();
  const auto fresh_pose_slack_s = node->get_parameter("fresh_pose_slack_s").as_double();
  const auto pose_sample_window_s = node->get_parameter("pose_sample_window_s").as_double();
  const auto pose_sample_count = std::max<int64_t>(1, node->get_parameter("pose_sample_count").as_int());
  const auto min_pose_sample_count = std::max<int64_t>(1, node->get_parameter("min_pose_sample_count").as_int());
  const auto max_pose_spread_m = node->get_parameter("max_pose_spread_m").as_double();
  const auto post_rest_settle_s = node->get_parameter("post_rest_settle_s").as_double();
  const auto pose_transform_timeout_s = node->get_parameter("pose_transform_timeout_s").as_double();
  const auto pregrasp_offset_m = node->get_parameter("pregrasp_offset_m").as_double();
  const auto hover_high_offset_m = node->get_parameter("hover_high_offset_m").as_double();
  const auto pregrasp_x_offset_m = node->get_parameter("pregrasp_x_offset_m").as_double();
  const auto pregrasp_y_offset_m = node->get_parameter("pregrasp_y_offset_m").as_double();
  const auto pregrasp_z_extra_m = node->get_parameter("pregrasp_z_extra_m").as_double();
  const auto use_two_stage_pregrasp = node->get_parameter("use_two_stage_pregrasp").as_bool();
  const auto grasp_x_offset_m = node->get_parameter("grasp_x_offset_m").as_double();
  const auto grasp_y_offset_m = node->get_parameter("grasp_y_offset_m").as_double();
  const auto grasp_z_offset_m = node->get_parameter("grasp_z_offset_m").as_double();
  const auto enforce_table_z_limits = node->get_parameter("enforce_table_z_limits").as_bool();
  const auto tabletop_z_m = node->get_parameter("tabletop_z_m").as_double();
  const auto use_tabletop_z_topic = node->get_parameter("use_tabletop_z_topic").as_bool();
  const auto tabletop_z_topic = node->get_parameter("tabletop_z_topic").as_string();
  const auto wait_tabletop_z_timeout_s = std::max(0.0, node->get_parameter("wait_tabletop_z_timeout_s").as_double());
  const auto fresh_tabletop_z_slack_s = std::max(0.0, node->get_parameter("fresh_tabletop_z_slack_s").as_double());
  const auto min_grasp_clearance_m = node->get_parameter("min_grasp_clearance_m").as_double();
  const auto min_pregrasp_clearance_m = node->get_parameter("min_pregrasp_clearance_m").as_double();
  const auto add_table_collision = node->get_parameter("add_table_collision").as_bool();
  const auto table_center_x_m = node->get_parameter("table_center_x_m").as_double();
  const auto table_center_y_m = node->get_parameter("table_center_y_m").as_double();
  const auto table_size_x_m = node->get_parameter("table_size_x_m").as_double();
  const auto table_size_y_m = node->get_parameter("table_size_y_m").as_double();
  const auto table_thickness_m = node->get_parameter("table_thickness_m").as_double();
  const auto position_only = node->get_parameter("position_only").as_bool();
  const auto go_to_rest_before_grasp = node->get_parameter("go_to_rest_before_grasp").as_bool();
  const auto staging_named_pose = node->get_parameter("staging_named_pose").as_string();
  const auto return_to_named_pose_after_grasp = node->get_parameter("return_to_named_pose_after_grasp").as_bool();
  const auto post_grasp_named_pose = node->get_parameter("post_grasp_named_pose").as_string();
  const auto open_gripper_after_return = node->get_parameter("open_gripper_after_return").as_bool();
  const auto post_return_gripper_settle_s = node->get_parameter("post_return_gripper_settle_s").as_double();
  const auto post_grasp_lift_before_return = node->get_parameter("post_grasp_lift_before_return").as_bool();
  const auto post_grasp_return_lift_z_m = node->get_parameter("post_grasp_return_lift_z_m").as_double();
  const auto post_grasp_return_lift_extra_m = node->get_parameter("post_grasp_return_lift_extra_m").as_double();
  const auto ik_post_grasp_lift_strategy = node->get_parameter("ik_post_grasp_lift_strategy").as_string();
  const auto ik_post_grasp_lift_duration_s = node->get_parameter("ik_post_grasp_lift_duration_s").as_double();
  const auto ik_post_grasp_lift_settle_s = node->get_parameter("ik_post_grasp_lift_settle_s").as_double();
  const auto post_grasp_return_via_rest_hover =
    node->get_parameter("post_grasp_return_via_rest_hover").as_bool();
  const auto post_grasp_rest_hover_x_m = node->get_parameter("post_grasp_rest_hover_x_m").as_double();
  const auto post_grasp_rest_hover_y_m = node->get_parameter("post_grasp_rest_hover_y_m").as_double();
  const auto post_grasp_rest_hover_z_m = node->get_parameter("post_grasp_rest_hover_z_m").as_double();
  const auto ik_post_grasp_rest_hover_strategy =
    node->get_parameter("ik_post_grasp_rest_hover_strategy").as_string();
  const auto ik_post_grasp_rest_hover_duration_s =
    node->get_parameter("ik_post_grasp_rest_hover_duration_s").as_double();
  const auto ik_post_grasp_rest_hover_settle_s =
    node->get_parameter("ik_post_grasp_rest_hover_settle_s").as_double();
  const auto post_grasp_use_ik_joints = node->get_parameter("post_grasp_use_ik_joints").as_bool();
  const auto ik_joints_service_name = node->get_parameter("ik_joints_service_name").as_string();
  const auto ik_post_grasp_duration_s = node->get_parameter("ik_post_grasp_duration_s").as_double();
  const auto use_marker_orientation = node->get_parameter("use_marker_orientation").as_bool();
  const auto use_rpy = node->get_parameter("use_rpy").as_bool();
  const auto qx = node->get_parameter("qx").as_double();
  const auto qy = node->get_parameter("qy").as_double();
  const auto qz = node->get_parameter("qz").as_double();
  const auto qw = node->get_parameter("qw").as_double();
  const auto roll = node->get_parameter("roll").as_double();
  const auto pitch = node->get_parameter("pitch").as_double();
  const auto yaw = node->get_parameter("yaw").as_double();
  const auto planning_time_s = node->get_parameter("planning_time_s").as_double();
  const auto vel_scaling = node->get_parameter("vel_scaling").as_double();
  const auto acc_scaling = node->get_parameter("acc_scaling").as_double();
  const auto use_ik_pose_service = node->get_parameter("use_ik_pose_service").as_bool();
  const auto ik_pose_service_name = node->get_parameter("ik_pose_service_name").as_string();
  const auto ik_pose_strategy = node->get_parameter("ik_pose_strategy").as_string();
  const auto ik_pregrasp_strategy = node->get_parameter("ik_pregrasp_strategy").as_string();
  const auto ik_grasp_strategy = node->get_parameter("ik_grasp_strategy").as_string();
  const auto ik_retreat_strategy = node->get_parameter("ik_retreat_strategy").as_string();
  const auto ik_pregrasp_duration_s = node->get_parameter("ik_pregrasp_duration_s").as_double();
  const auto ik_grasp_duration_s = node->get_parameter("ik_grasp_duration_s").as_double();
  const auto ik_retreat_duration_s = node->get_parameter("ik_retreat_duration_s").as_double();
  const auto ik_pose_service_timeout_s = node->get_parameter("ik_pose_service_timeout_s").as_double();
  const auto ik_pregrasp_settle_s = node->get_parameter("ik_pregrasp_settle_s").as_double();
  const auto ik_grasp_settle_s = node->get_parameter("ik_grasp_settle_s").as_double();
  const auto ik_retreat_settle_s = node->get_parameter("ik_retreat_settle_s").as_double();
  const auto gripper_action_name = node->get_parameter("gripper_action_name").as_string();
  const auto gripper_open_pos = node->get_parameter("gripper_open_pos").as_double();
  const auto gripper_closed_pos = node->get_parameter("gripper_closed_pos").as_double();
  const auto gripper_max_effort = node->get_parameter("gripper_max_effort").as_double();
  const auto action_timeout_s = node->get_parameter("action_timeout_s").as_double();
  const auto grasp_retry_count = std::max<int64_t>(0, node->get_parameter("grasp_retry_count").as_int());
  const auto grasp_retry_delay_s = std::max(0.0, node->get_parameter("grasp_retry_delay_s").as_double());
  const auto post_grasp_return_retry_count =
    std::max<int64_t>(0, node->get_parameter("post_grasp_return_retry_count").as_int());
  const auto post_grasp_return_retry_delay_s =
    std::max(0.0, node->get_parameter("post_grasp_return_retry_delay_s").as_double());

  if (!ensure_robot_model_params_from_move_group(node, move_group_node)) {
    rclcpp::shutdown();
    return 2;
  }

  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer, node, false);

  struct PoseState
  {
    std::mutex mutex;
    std::condition_variable cv;
    std::optional<geometry_msgs::msg::PoseStamped> latest;
    std::deque<geometry_msgs::msg::PoseStamped> samples;
    std::optional<rclcpp::Time> min_stamp;
    std::size_t update_count{0};
  } pose_state;

  struct TableZState
  {
    std::mutex mutex;
    std::condition_variable cv;
    std::optional<double> latest;
    std::optional<rclcpp::Time> latest_stamp;
  } table_z_state;

  auto pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic,
    10,
    [&](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(pose_state.mutex);
      if (pose_state.min_stamp.has_value()) {
        const rclcpp::Time stamp(msg->header.stamp);
        if (stamp < pose_state.min_stamp.value()) {
          return;
        }
      }
      pose_state.latest = *msg;
      pose_state.samples.push_back(*msg);
      const std::size_t max_samples = static_cast<std::size_t>(std::max<int64_t>(pose_sample_count, 1) * 4);
      while (pose_state.samples.size() > max_samples) {
        pose_state.samples.pop_front();
      }
      pose_state.update_count++;
      pose_state.cv.notify_all();
    });

  (void)pose_sub;

  auto table_z_sub = node->create_subscription<std_msgs::msg::Float64>(
    tabletop_z_topic,
    10,
    [&](const std_msgs::msg::Float64::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(table_z_state.mutex);
      table_z_state.latest = msg->data;
      table_z_state.latest_stamp = node->now();
      table_z_state.cv.notify_all();
    });

  (void)table_z_sub;

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  std::thread spin_thread([&exec]() { exec.spin(); });

  auto fail = [&](const std::string& what) {
    RCLCPP_ERROR(node->get_logger(), "%s", what.c_str());
    exec.cancel();
    rclcpp::shutdown();
    if (spin_thread.joinable()) spin_thread.join();
    return 1;
  };

  moveit::planning_interface::MoveGroupInterface arm(node, arm_group);
  if (!ik_tip_link.empty()) {
    const bool ee_ok = arm.setEndEffectorLink(ik_tip_link);
    if (!ee_ok) {
      RCLCPP_WARN(
        node->get_logger(),
        "Failed to set end-effector link to '%s'. MoveIt will keep using '%s'.",
        ik_tip_link.c_str(),
        arm.getEndEffectorLink().c_str());
    }
  }
  arm.setPoseReferenceFrame(target_frame);
  arm.setPlanningTime(planning_time_s);
  arm.setMaxVelocityScalingFactor(vel_scaling);
  arm.setMaxAccelerationScalingFactor(acc_scaling);
  RCLCPP_INFO(
    node->get_logger(),
    "MoveGroup configured: planning_frame='%s' pose_frame='%s' ee_link='%s'",
    arm.getPlanningFrame().c_str(),
    target_frame.c_str(),
    arm.getEndEffectorLink().c_str());
  log_current_ee_pose(arm, node->get_logger(), "Current");

  auto move_to_pose_target =
    [&](const geometry_msgs::msg::PoseStamped& target,
        double service_duration_s,
        const std::string& service_strategy,
        double settle_s,
        const char* stage_name) {
      if (use_ik_pose_service && execute) {
        const std::string effective_strategy =
          service_strategy.empty() ? ik_pose_strategy : service_strategy;
        const double effective_duration_s =
          (effective_strategy == "cartesian") ? 0.0 : service_duration_s;
        RCLCPP_INFO(
          node->get_logger(),
          "Moving to %s via IK pose service '%s' (strategy=%s, duration=%.2fs)...",
          stage_name,
          ik_pose_service_name.c_str(),
          effective_strategy.c_str(),
          effective_duration_s);
        const bool ok = send_go_to_pose_request(
          node,
          ik_pose_service_name,
          target,
          effective_strategy,
          effective_duration_s,
          ik_pose_service_timeout_s);
        if (!ok) return false;
        if (settle_s > 0.0) {
          RCLCPP_INFO(
            node->get_logger(),
            "Waiting %.2fs for %s settle...",
            settle_s,
            stage_name);
          rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(settle_s)));
        }
        return true;
      }

      RCLCPP_INFO(
        node->get_logger(),
        "Moving to %s via MoveIt pose planning...",
        stage_name);
      return plan_and_execute_pose(arm, target, execute, position_only, node->get_logger());
    };

  auto resolve_tabletop_z = [&]() {
    double resolved_tabletop_z_m = tabletop_z_m;
    if (!use_tabletop_z_topic) {
      RCLCPP_INFO(
        node->get_logger(),
        "Using static tabletop_z_m=%.3f (topic disabled).",
        resolved_tabletop_z_m);
      return resolved_tabletop_z_m;
    }

    std::unique_lock<std::mutex> lock(table_z_state.mutex);
    const auto min_stamp =
      node->now() - rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(fresh_tabletop_z_slack_s)));
    const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(wait_tabletop_z_timeout_s);
    while (rclcpp::ok()) {
      const bool fresh =
        table_z_state.latest.has_value() &&
        table_z_state.latest_stamp.has_value() &&
        table_z_state.latest_stamp.value() >= min_stamp;
      if (fresh) {
        resolved_tabletop_z_m = table_z_state.latest.value();
        break;
      }
      if (table_z_state.cv.wait_until(lock, deadline) == std::cv_status::timeout) {
        break;
      }
    }
    if (resolved_tabletop_z_m != tabletop_z_m) {
      RCLCPP_INFO(
        node->get_logger(),
        "Using tabletop z from topic '%s': %.3f m (static fallback %.3f m).",
        tabletop_z_topic.c_str(),
        resolved_tabletop_z_m,
        tabletop_z_m);
    } else {
      RCLCPP_WARN(
        node->get_logger(),
        "No fresh tabletop z received on '%s' within %.2fs; using static tabletop_z_m=%.3f.",
        tabletop_z_topic.c_str(),
        wait_tabletop_z_timeout_s,
        tabletop_z_m);
    }
    return resolved_tabletop_z_m;
  };

  const double effective_tabletop_z_m = resolve_tabletop_z();

  if (add_table_collision) {
    (void)apply_table_collision(
      node,
      target_frame,
      table_center_x_m,
      table_center_y_m,
      effective_tabletop_z_m,
      table_size_x_m,
      table_size_y_m,
      table_thickness_m);
  }

  auto wait_for_stable_pose = [&](bool require_fresh) -> std::optional<geometry_msgs::msg::PoseStamped> {
    std::vector<geometry_msgs::msg::PoseStamped> pose_samples;
    std::optional<geometry_msgs::msg::PoseStamped> pose_opt;
    {
      std::unique_lock<std::mutex> lock(pose_state.mutex);
      if (require_fresh) {
        pose_state.latest.reset();
        pose_state.samples.clear();
        const auto slack_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(std::max(0.0, fresh_pose_slack_s)));
        pose_state.min_stamp = node->now() - rclcpp::Duration(slack_ns);
      }
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(wait_pose_timeout_s);
      while (!pose_state.latest.has_value()) {
        if (pose_state.cv.wait_until(lock, deadline) == std::cv_status::timeout) {
          break;
        }
      }
      if (pose_state.latest.has_value()) {
        pose_opt = pose_state.latest;
        const auto sample_deadline =
          std::chrono::steady_clock::now() + std::chrono::duration<double>(pose_sample_window_s);
        const auto start_update_count = pose_state.update_count;
        while (std::chrono::steady_clock::now() < sample_deadline &&
               static_cast<int64_t>(pose_state.samples.size()) < pose_sample_count)
        {
          if (pose_state.cv.wait_until(lock, sample_deadline) == std::cv_status::timeout) {
            break;
          }
          if (pose_state.update_count == start_update_count) {
            continue;
          }
        }

        const auto available = static_cast<int64_t>(pose_state.samples.size());
        const auto keep = std::max<int64_t>(1, std::min<int64_t>(pose_sample_count, available));
        if (available < std::min(pose_sample_count, min_pose_sample_count)) {
          RCLCPP_ERROR(
            node->get_logger(),
            "Only %ld pose samples available on '%s' (need at least %ld).",
            static_cast<long>(available),
            pose_topic.c_str(),
            static_cast<long>(std::min(pose_sample_count, min_pose_sample_count)));
          return std::nullopt;
        }
        pose_samples.reserve(static_cast<std::size_t>(keep));
        for (auto it = pose_state.samples.end() - keep; it != pose_state.samples.end(); ++it) {
          pose_samples.push_back(*it);
        }
      }
    }

    if (!pose_opt.has_value()) {
      return std::nullopt;
    }

    geometry_msgs::msg::PoseStamped grasp = pose_opt.value();
    if (!pose_samples.empty()) {
      std::vector<double> xs;
      std::vector<double> ys;
      std::vector<double> zs;
      xs.reserve(pose_samples.size());
      ys.reserve(pose_samples.size());
      zs.reserve(pose_samples.size());
      for (const auto& sample : pose_samples) {
        xs.push_back(sample.pose.position.x);
        ys.push_back(sample.pose.position.y);
        zs.push_back(sample.pose.position.z);
      }

      const double median_x = median_of(xs);
      const double median_y = median_of(ys);
      const double median_z = median_of(zs);
      double max_spread = 0.0;
      for (const auto& sample : pose_samples) {
        const double dx = sample.pose.position.x - median_x;
        const double dy = sample.pose.position.y - median_y;
        const double dz = sample.pose.position.z - median_z;
        max_spread = std::max(max_spread, std::sqrt(dx * dx + dy * dy + dz * dz));
      }
      if (max_pose_spread_m > 0.0 && max_spread > max_pose_spread_m) {
        RCLCPP_ERROR(
          node->get_logger(),
          "Pose samples are unstable on '%s' (max spread=%.3fm).",
          pose_topic.c_str(),
          max_spread);
        return std::nullopt;
      }
      grasp = pose_samples.back();
      grasp.pose.position.x = median_x;
      grasp.pose.position.y = median_y;
      grasp.pose.position.z = median_z;
      RCLCPP_INFO(
        node->get_logger(),
        "Using %zu pose samples (window=%.2fs, spread=%.3fm).",
        pose_samples.size(),
        pose_sample_window_s,
        max_spread);
    }

    if (grasp.header.frame_id != target_frame) {
      try {
        grasp = tf_buffer.transform(
          grasp,
          target_frame,
          tf2::durationFromSec(pose_transform_timeout_s));
      } catch (const std::exception& exc) {
        RCLCPP_ERROR(
          node->get_logger(),
          "Failed to transform target pose to %s: %s",
          target_frame.c_str(),
          exc.what());
        return std::nullopt;
      }
    }

    if (!use_marker_orientation) {
      if (use_rpy) {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q.normalize();
        grasp.pose.orientation.x = q.x();
        grasp.pose.orientation.y = q.y();
        grasp.pose.orientation.z = q.z();
        grasp.pose.orientation.w = q.w();
      } else {
        const bool all_zero = (std::fabs(qx) < 1e-12) && (std::fabs(qy) < 1e-12) &&
                              (std::fabs(qz) < 1e-12) && (std::fabs(qw) < 1e-12);
        grasp.pose.orientation.x = all_zero ? 0.0 : qx;
        grasp.pose.orientation.y = all_zero ? 0.0 : qy;
        grasp.pose.orientation.z = all_zero ? 0.0 : qz;
        grasp.pose.orientation.w = all_zero ? 1.0 : qw;
      }
    }

    grasp.pose.position.x += grasp_x_offset_m;
    grasp.pose.position.y += grasp_y_offset_m;
    grasp.pose.position.z += grasp_z_offset_m;
    return grasp;
  };

  if (enable_gripper && execute) {
    RCLCPP_INFO(node->get_logger(), "Opening gripper...");
    if (!send_gripper_command(node, gripper_action_name, gripper_open_pos, gripper_max_effort, action_timeout_s)) {
      return fail("Failed to open gripper.");
    }
  } else if (enable_gripper && !execute) {
    RCLCPP_INFO(node->get_logger(), "Dry-run mode: skip gripper open action.");
  }

  if (enable_arm && go_to_rest_before_grasp) {
    RCLCPP_INFO(node->get_logger(), "Moving to staging pose '%s' first...", staging_named_pose.c_str());
    if (!plan_and_execute_named(arm, staging_named_pose, execute, node->get_logger())) {
      return fail(std::string("Failed to move to staging pose '") + staging_named_pose + "'.");
    }
    if (execute && post_rest_settle_s > 0.0) {
      RCLCPP_INFO(node->get_logger(), "Waiting %.2fs for vision to settle after staging...", post_rest_settle_s);
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(post_rest_settle_s)));
    }
  } else {
    RCLCPP_INFO(
      node->get_logger(),
      "Skipping named staging pose. The arm will plan directly from its current state to the target pregrasp.");
  }

  geometry_msgs::msg::PoseStamped grasp;
  geometry_msgs::msg::PoseStamped pregrasp;
  geometry_msgs::msg::PoseStamped hover_high;
  bool reached_grasp = !enable_arm;
  std::string last_grasp_attempt_error;
  const int64_t max_grasp_attempts = grasp_retry_count + 1;

  for (int64_t attempt = 1; attempt <= max_grasp_attempts; ++attempt) {
    RCLCPP_INFO(
      node->get_logger(),
      "Grasp planning attempt %ld/%ld: refreshing current arm state and target pose...",
      static_cast<long>(attempt),
      static_cast<long>(max_grasp_attempts));
    log_current_ee_pose(arm, node->get_logger(), "Attempt start");

    auto grasp_opt = wait_for_stable_pose(true);
    if (!grasp_opt.has_value()) {
      last_grasp_attempt_error =
        std::string("Timed out waiting for a fresh stable pose on '") + pose_topic +
        "' (fresh_pose_slack_s=" + std::to_string(fresh_pose_slack_s) + ").";
      RCLCPP_WARN(node->get_logger(), "%s", last_grasp_attempt_error.c_str());
    } else {
      grasp = grasp_opt.value();
      pregrasp = grasp;
      hover_high = grasp;
      pregrasp.pose.position.x += pregrasp_x_offset_m;
      pregrasp.pose.position.y += pregrasp_y_offset_m;
      pregrasp.pose.position.z += pregrasp_offset_m;
      pregrasp.pose.position.z += pregrasp_z_extra_m;
      hover_high.pose.position.x = pregrasp.pose.position.x;
      hover_high.pose.position.y = pregrasp.pose.position.y;
      hover_high.pose.position.z = pregrasp.pose.position.z + std::max(0.0, hover_high_offset_m);

      if (enforce_table_z_limits) {
        const double min_grasp_z_m = effective_tabletop_z_m + min_grasp_clearance_m;
        const double min_pregrasp_z_m = effective_tabletop_z_m + min_pregrasp_clearance_m;
        if (grasp.pose.position.z < min_grasp_z_m) {
          RCLCPP_WARN(
            node->get_logger(),
            "Clamping grasp z from %.3f m to tabletop-safe %.3f m (tabletop_z=%.3f, clearance=%.3f).",
            grasp.pose.position.z,
            min_grasp_z_m,
            effective_tabletop_z_m,
            min_grasp_clearance_m);
          grasp.pose.position.z = min_grasp_z_m;
        }
        if (pregrasp.pose.position.z < min_pregrasp_z_m) {
          RCLCPP_WARN(
            node->get_logger(),
            "Clamping pregrasp z from %.3f m to tabletop-safe %.3f m (tabletop_z=%.3f, clearance=%.3f).",
            pregrasp.pose.position.z,
            min_pregrasp_z_m,
            effective_tabletop_z_m,
            min_pregrasp_clearance_m);
          pregrasp.pose.position.z = min_pregrasp_z_m;
        }
        if (pregrasp.pose.position.z <= grasp.pose.position.z) {
          const double bumped_pregrasp_z = grasp.pose.position.z + std::max(0.01, pregrasp_offset_m);
          RCLCPP_WARN(
            node->get_logger(),
            "Bumping pregrasp z from %.3f m to %.3f m so it stays above grasp.",
            pregrasp.pose.position.z,
            bumped_pregrasp_z);
          pregrasp.pose.position.z = bumped_pregrasp_z;
        }
        if (hover_high.pose.position.z < pregrasp.pose.position.z) {
          hover_high.pose.position.z = pregrasp.pose.position.z;
        }
      }

      RCLCPP_INFO(
        node->get_logger(),
        "Target pose: frame='%s' x=%.3f y=%.3f z=%.3f",
        grasp.header.frame_id.c_str(),
        grasp.pose.position.x,
        grasp.pose.position.y,
        grasp.pose.position.z);
      RCLCPP_INFO(
        node->get_logger(),
        "Pregrasp pose: x=%.3f y=%.3f z=%.3f",
        pregrasp.pose.position.x,
        pregrasp.pose.position.y,
        pregrasp.pose.position.z);
      RCLCPP_INFO(
        node->get_logger(),
        "Hover-high pose: x=%.3f y=%.3f z=%.3f (enabled=%s)",
        hover_high.pose.position.x,
        hover_high.pose.position.y,
        hover_high.pose.position.z,
        use_two_stage_pregrasp ? "true" : "false");
      RCLCPP_INFO(
        node->get_logger(),
        "Orientation mode: marker=%s rpy=%s quat=[%.3f, %.3f, %.3f, %.3f] rpy=[%.3f, %.3f, %.3f]",
        use_marker_orientation ? "true" : "false",
        use_rpy ? "true" : "false",
        grasp.pose.orientation.x,
        grasp.pose.orientation.y,
        grasp.pose.orientation.z,
        grasp.pose.orientation.w,
        roll,
        pitch,
        yaw);

      bool attempt_ok = true;
      if (enable_arm) {
        if (use_two_stage_pregrasp && hover_high_offset_m > 1e-6) {
          if (!move_to_pose_target(
                hover_high,
                ik_pregrasp_duration_s,
                ik_pregrasp_strategy,
                ik_pregrasp_settle_s,
                "hover_high")) {
            last_grasp_attempt_error = "Failed to move to hover_high.";
            attempt_ok = false;
          } else {
            log_current_ee_pose(arm, node->get_logger(), "After hover_high", hover_high);
          }
        }
        if (attempt_ok) {
          if (!move_to_pose_target(
                pregrasp,
                ik_pregrasp_duration_s,
                ik_pregrasp_strategy,
                ik_pregrasp_settle_s,
                "pregrasp")) {
            last_grasp_attempt_error = "Failed to move to pregrasp.";
            attempt_ok = false;
          } else {
            log_current_ee_pose(arm, node->get_logger(), "After pregrasp", pregrasp);
          }
        }
        if (attempt_ok) {
          if (!move_to_pose_target(
                grasp,
                ik_grasp_duration_s,
                ik_grasp_strategy,
                ik_grasp_settle_s,
                "grasp")) {
            last_grasp_attempt_error = "Failed to move to grasp.";
            attempt_ok = false;
          } else {
            log_current_ee_pose(arm, node->get_logger(), "After grasp", grasp);
          }
        }
      }

      if (attempt_ok) {
        reached_grasp = true;
        break;
      }
      RCLCPP_WARN(
        node->get_logger(),
        "Grasp planning attempt %ld/%ld failed: %s",
        static_cast<long>(attempt),
        static_cast<long>(max_grasp_attempts),
        last_grasp_attempt_error.c_str());
    }

    if (attempt < max_grasp_attempts && grasp_retry_delay_s > 0.0) {
      RCLCPP_INFO(
        node->get_logger(),
        "Waiting %.2fs before retrying with fresh arm state and target pose...",
        grasp_retry_delay_s);
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(grasp_retry_delay_s)));
    }
  }

  if (!reached_grasp) {
    return fail(
      last_grasp_attempt_error.empty()
        ? std::string("Failed to reach grasp after retries.")
        : last_grasp_attempt_error);
  }

  if (enable_gripper && execute) {
    RCLCPP_INFO(node->get_logger(), "Closing gripper...");
    if (!send_gripper_command(node, gripper_action_name, gripper_closed_pos, gripper_max_effort, action_timeout_s)) {
      return fail("Failed to close gripper.");
    }
  }

  if (enable_arm) {
    RCLCPP_INFO(node->get_logger(), "Retreating...");
    if (!move_to_pose_target(
          pregrasp,
          ik_retreat_duration_s,
          ik_retreat_strategy,
          ik_retreat_settle_s,
          "retreat")) {
      return fail("Failed to retreat.");
    }
    log_current_ee_pose(arm, node->get_logger(), "After retreat", pregrasp);
    if (return_to_named_pose_after_grasp) {
      if (post_grasp_lift_before_return) {
        auto return_lift = pregrasp;
        return_lift.pose.position.z = std::max(
          pregrasp.pose.position.z + std::max(0.0, post_grasp_return_lift_extra_m),
          post_grasp_return_lift_z_m);
        if (enforce_table_z_limits) {
          return_lift.pose.position.z = std::max(
            return_lift.pose.position.z,
            effective_tabletop_z_m + min_pregrasp_clearance_m);
        }

        RCLCPP_INFO(
          node->get_logger(),
          "Lifting carried object before returning to '%s': x=%.3f y=%.3f z=%.3f...",
          post_grasp_named_pose.c_str(),
          return_lift.pose.position.x,
          return_lift.pose.position.y,
          return_lift.pose.position.z);
        if (!move_to_pose_target(
              return_lift,
              ik_post_grasp_lift_duration_s,
              ik_post_grasp_lift_strategy,
              ik_post_grasp_lift_settle_s,
              "post_grasp_return_lift")) {
          return fail("Failed to lift before returning to post-grasp pose.");
        }
        log_current_ee_pose(arm, node->get_logger(), "After post-grasp return lift", return_lift);
      }

      if (post_grasp_return_via_rest_hover) {
        auto rest_hover = pregrasp;
        rest_hover.pose.position.x = post_grasp_rest_hover_x_m;
        rest_hover.pose.position.y = post_grasp_rest_hover_y_m;
        rest_hover.pose.position.z = post_grasp_rest_hover_z_m;
        if (enforce_table_z_limits) {
          rest_hover.pose.position.z = std::max(
            rest_hover.pose.position.z,
            effective_tabletop_z_m + min_pregrasp_clearance_m);
        }

        RCLCPP_INFO(
          node->get_logger(),
          "Moving carried object to safe rest-hover before '%s': x=%.3f y=%.3f z=%.3f...",
          post_grasp_named_pose.c_str(),
          rest_hover.pose.position.x,
          rest_hover.pose.position.y,
          rest_hover.pose.position.z);
        if (!move_to_pose_target(
              rest_hover,
              ik_post_grasp_rest_hover_duration_s,
              ik_post_grasp_rest_hover_strategy,
              ik_post_grasp_rest_hover_settle_s,
              "post_grasp_rest_hover")) {
          return fail("Failed to move to safe rest-hover before post-grasp pose.");
        }
        log_current_ee_pose(arm, node->get_logger(), "After post-grasp rest-hover", rest_hover);
      }

      RCLCPP_INFO(node->get_logger(), "Returning to post-grasp pose '%s'...", post_grasp_named_pose.c_str());
      bool returned_to_post_grasp_pose = false;
      const int64_t max_return_attempts = post_grasp_return_retry_count + 1;
      for (int64_t return_attempt = 1; return_attempt <= max_return_attempts; ++return_attempt) {
        RCLCPP_INFO(
          node->get_logger(),
          "Post-grasp return attempt %ld/%ld: refreshing current arm state...",
          static_cast<long>(return_attempt),
          static_cast<long>(max_return_attempts));
        log_current_ee_pose(arm, node->get_logger(), "Post-grasp return attempt start");

        if (post_grasp_use_ik_joints && execute) {
          std::vector<std::string> joint_names;
          std::vector<double> positions;
          if (named_pose_to_manipulator_joints(post_grasp_named_pose, joint_names, positions)) {
            RCLCPP_INFO(
              node->get_logger(),
              "Returning to '%s' via IK joints service '%s'...",
              post_grasp_named_pose.c_str(),
              ik_joints_service_name.c_str());
            returned_to_post_grasp_pose = send_go_to_joints_request(
              node,
              ik_joints_service_name,
              joint_names,
              positions,
              ik_post_grasp_duration_s,
              ik_pose_service_timeout_s);
            if (!returned_to_post_grasp_pose) {
              RCLCPP_WARN(
                node->get_logger(),
                "IK joints return to '%s' failed; falling back to MoveIt named target.",
                post_grasp_named_pose.c_str());
            }
          } else {
            RCLCPP_WARN(
              node->get_logger(),
              "Post-grasp named pose '%s' has no IK joints mapping; falling back to MoveIt named target.",
              post_grasp_named_pose.c_str());
          }
        }

        if (!returned_to_post_grasp_pose) {
          returned_to_post_grasp_pose =
            plan_and_execute_named(arm, post_grasp_named_pose, execute, node->get_logger());
        }

        if (returned_to_post_grasp_pose) {
          break;
        }

        if (return_attempt < max_return_attempts && post_grasp_return_retry_delay_s > 0.0) {
          RCLCPP_WARN(
            node->get_logger(),
            "Post-grasp return attempt %ld/%ld failed; waiting %.2fs before retrying from current state.",
            static_cast<long>(return_attempt),
            static_cast<long>(max_return_attempts),
            post_grasp_return_retry_delay_s);
          rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(post_grasp_return_retry_delay_s)));
        }
      }
      if (!returned_to_post_grasp_pose) {
        return fail(std::string("Failed to return to post-grasp pose '") + post_grasp_named_pose + "'.");
      }
      log_current_ee_pose(arm, node->get_logger(), "After post-grasp named pose");
      if (enable_gripper && execute && open_gripper_after_return) {
        RCLCPP_INFO(
          node->get_logger(),
          "Opening gripper after returning to '%s'...",
          post_grasp_named_pose.c_str());
        if (!send_gripper_command(node, gripper_action_name, gripper_open_pos, gripper_max_effort, action_timeout_s)) {
          return fail("Failed to open gripper after returning to post-grasp pose.");
        }
        if (post_return_gripper_settle_s > 0.0) {
          RCLCPP_INFO(
            node->get_logger(),
            "Waiting %.2fs after post-return gripper open...",
            post_return_gripper_settle_s);
          rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(post_return_gripper_settle_s)));
        }
      }
    }
  }

  RCLCPP_INFO(node->get_logger(), "Visual grasp sequence completed.");

  exec.cancel();
  rclcpp::shutdown();
  if (spin_thread.joinable()) spin_thread.join();
  return 0;
}
