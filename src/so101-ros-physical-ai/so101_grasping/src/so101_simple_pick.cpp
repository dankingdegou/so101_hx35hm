#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/gripper_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <thread>
#include <vector>

using GripperCommand = control_msgs::action::GripperCommand;

static bool ensure_robot_model_params_from_move_group(
  const rclcpp::Node::SharedPtr& node,
  const std::string& move_group_node)
{
  // MoveGroupInterface requires these parameters on *this* node.
  // We fetch them from the running move_group node and set them locally.
  if (!node) return false;

  // If already injected from launch/CLI, keep it.
  try {
    const auto urdf = node->get_parameter("robot_description").as_string();
    const auto srdf = node->get_parameter("robot_description_semantic").as_string();
    if (!urdf.empty() && !srdf.empty()) return true;
  } catch (...) {
    // ignore
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

  // Also copy kinematics + joint limits namespaces so this node can solve IK for pose targets.
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
      // This node is created with allow_undeclared_parameters(true), so direct set is OK.
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
  auto client = rclcpp_action::create_client<GripperCommand>(node, action_name);
  if (!client->wait_for_action_server(std::chrono::duration<double>(timeout_s))) {
    RCLCPP_ERROR(node->get_logger(), "Gripper action server not available: %s", action_name.c_str());
    return false;
  }

  GripperCommand::Goal goal;
  goal.command.position = position;
  goal.command.max_effort = max_effort;

  auto send_fut = client->async_send_goal(goal);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  if (exec.spin_until_future_complete(send_fut, std::chrono::duration<double>(timeout_s)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    exec.remove_node(node);
    RCLCPP_ERROR(node->get_logger(), "Timed out sending gripper goal.");
    return false;
  }

  auto goal_handle = send_fut.get();
  if (!goal_handle) {
    exec.remove_node(node);
    RCLCPP_ERROR(node->get_logger(), "Gripper goal rejected.");
    return false;
  }

  auto res_fut = client->async_get_result(goal_handle);
  if (exec.spin_until_future_complete(res_fut, std::chrono::duration<double>(timeout_s)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    exec.remove_node(node);
    RCLCPP_ERROR(node->get_logger(), "Timed out waiting for gripper result.");
    return false;
  }
  exec.remove_node(node);

  const auto wrapped = res_fut.get();
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(node->get_logger(), "Gripper action failed (code=%d).", static_cast<int>(wrapped.code));
    return false;
  }
  return true;
}

static bool plan_and_execute_pose(
  moveit::planning_interface::MoveGroupInterface& group,
  const geometry_msgs::msg::PoseStamped& target,
  bool execute,
  rclcpp::Logger logger)
{
  group.setPoseTarget(target);

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

// Minimal "pick" sequence:
// 1) Open gripper (named target "open" from SRDF)
// 2) Move arm to pregrasp (x,y,z+offset)
// 3) Move arm to grasp (x,y,z)
// 4) Close gripper (named target "closed" from SRDF)
// 5) Retreat back to pregrasp
//
// This does NOT do perception. You provide a grasp pose in the planning frame (default: base_link).
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "so101_simple_pick",
    rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true));

  auto declare_if_missing = [&](auto /*tag*/, const std::string& name, auto def) {
    if (!node->has_parameter(name)) node->declare_parameter(name, def);
  };

  declare_if_missing(std::string{}, "move_group_node", std::string{"/move_group"});
  declare_if_missing(std::string{}, "arm_group", std::string{"manipulator"});
  declare_if_missing(std::string{}, "gripper_group", std::string{"gripper"});
  declare_if_missing(std::string{}, "target_frame", std::string{"base_link"});
  declare_if_missing(bool{}, "execute", true);
  declare_if_missing(bool{}, "enable_arm", true);
  declare_if_missing(bool{}, "enable_gripper", true);

  // Gripper is controlled via an action by default (more robust than planning a 1-DOF group).
  declare_if_missing(std::string{}, "gripper_action_name", std::string{"/follower/gripper_controller/gripper_cmd"});
  declare_if_missing(double{}, "gripper_open_pos", 1.5);
  declare_if_missing(double{}, "gripper_closed_pos", -0.16);
  declare_if_missing(double{}, "gripper_max_effort", 0.0);
  declare_if_missing(double{}, "action_timeout_s", 10.0);

  declare_if_missing(double{}, "x", 0.25);
  declare_if_missing(double{}, "y", 0.0);
  declare_if_missing(double{}, "z", 0.08);
  declare_if_missing(double{}, "qx", 0.0);
  declare_if_missing(double{}, "qy", 1.0);
  declare_if_missing(double{}, "qz", 0.0);
  declare_if_missing(double{}, "qw", 0.0);

  // Optional orientation as RPY (radians). If use_rpy=true, overrides qx/qy/qz/qw.
  declare_if_missing(bool{}, "use_rpy", false);
  declare_if_missing(double{}, "roll", 0.0);
  declare_if_missing(double{}, "pitch", 0.0);
  declare_if_missing(double{}, "yaw", 0.0);

  declare_if_missing(double{}, "pregrasp_offset_m", 0.08);
  declare_if_missing(double{}, "planning_time_s", 3.0);
  declare_if_missing(double{}, "vel_scaling", 0.4);
  declare_if_missing(double{}, "acc_scaling", 0.4);

  const auto arm_group = node->get_parameter("arm_group").as_string();
  const auto gripper_group = node->get_parameter("gripper_group").as_string();
  const auto target_frame = node->get_parameter("target_frame").as_string();
  const auto move_group_node = node->get_parameter("move_group_node").as_string();
  const auto execute = node->get_parameter("execute").as_bool();
  const auto enable_arm = node->get_parameter("enable_arm").as_bool();
  const auto enable_gripper = node->get_parameter("enable_gripper").as_bool();

  const auto gripper_action_name = node->get_parameter("gripper_action_name").as_string();
  const auto gripper_open_pos = node->get_parameter("gripper_open_pos").as_double();
  const auto gripper_closed_pos = node->get_parameter("gripper_closed_pos").as_double();
  const auto gripper_max_effort = node->get_parameter("gripper_max_effort").as_double();
  const auto action_timeout_s = node->get_parameter("action_timeout_s").as_double();

  const auto x = node->get_parameter("x").as_double();
  const auto y = node->get_parameter("y").as_double();
  const auto z = node->get_parameter("z").as_double();
  const auto qx = node->get_parameter("qx").as_double();
  const auto qy = node->get_parameter("qy").as_double();
  const auto qz = node->get_parameter("qz").as_double();
  const auto qw = node->get_parameter("qw").as_double();

  const auto use_rpy = node->get_parameter("use_rpy").as_bool();
  const auto roll = node->get_parameter("roll").as_double();
  const auto pitch = node->get_parameter("pitch").as_double();
  const auto yaw = node->get_parameter("yaw").as_double();

  const auto pregrasp_offset = node->get_parameter("pregrasp_offset_m").as_double();
  const auto planning_time = node->get_parameter("planning_time_s").as_double();
  const auto vel_scaling = node->get_parameter("vel_scaling").as_double();
  const auto acc_scaling = node->get_parameter("acc_scaling").as_double();

  if (!ensure_robot_model_params_from_move_group(node, move_group_node)) {
    rclcpp::shutdown();
    return 2;
  }

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  std::thread spin_thread([&exec]() { exec.spin(); });

  // MoveGroupInterface is a client to move_group.
  moveit::planning_interface::MoveGroupInterface arm(node, arm_group);

  arm.setPoseReferenceFrame(target_frame);
  arm.setPlanningTime(planning_time);
  arm.setMaxVelocityScalingFactor(vel_scaling);
  arm.setMaxAccelerationScalingFactor(acc_scaling);

  geometry_msgs::msg::PoseStamped grasp;
  grasp.header.frame_id = target_frame;
  grasp.pose.position.x = x;
  grasp.pose.position.y = y;
  grasp.pose.position.z = z;
  if (use_rpy) {
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize();
    grasp.pose.orientation.x = q.x();
    grasp.pose.orientation.y = q.y();
    grasp.pose.orientation.z = q.z();
    grasp.pose.orientation.w = q.w();
  } else {
    // If user passes a zero quaternion, keep a sane default (identity).
    const bool all_zero = (std::fabs(qx) < 1e-12) && (std::fabs(qy) < 1e-12) && (std::fabs(qz) < 1e-12) &&
                          (std::fabs(qw) < 1e-12);
    grasp.pose.orientation.x = all_zero ? 0.0 : qx;
    grasp.pose.orientation.y = all_zero ? 0.0 : qy;
    grasp.pose.orientation.z = all_zero ? 0.0 : qz;
    grasp.pose.orientation.w = all_zero ? 1.0 : qw;
  }

  geometry_msgs::msg::PoseStamped pregrasp = grasp;
  pregrasp.pose.position.z = z + pregrasp_offset;

  auto fail = [&](const std::string& what) {
    RCLCPP_ERROR(node->get_logger(), "%s", what.c_str());
    exec.cancel();
    rclcpp::shutdown();
    if (spin_thread.joinable()) spin_thread.join();
    return 1;
  };

  // 1) Open gripper.
  if (enable_gripper) {
    RCLCPP_INFO(node->get_logger(), "Opening gripper (action: %s) ...", gripper_action_name.c_str());
    if (!send_gripper_command(node, gripper_action_name, gripper_open_pos, gripper_max_effort, action_timeout_s)) {
      return fail("Failed to open gripper.");
    }
  }

  // 2) Move to pregrasp.
  if (enable_arm) {
    RCLCPP_INFO(node->get_logger(), "Moving to pregrasp...");
    if (!plan_and_execute_pose(arm, pregrasp, execute, node->get_logger())) {
      return fail("Failed to move to pregrasp.");
    }
  }

  // 3) Move to grasp.
  if (enable_arm) {
    RCLCPP_INFO(node->get_logger(), "Moving to grasp...");
    if (!plan_and_execute_pose(arm, grasp, execute, node->get_logger())) {
      return fail("Failed to move to grasp.");
    }
  }

  // 4) Close gripper.
  if (enable_gripper) {
    RCLCPP_INFO(node->get_logger(), "Closing gripper (action: %s) ...", gripper_action_name.c_str());
    if (!send_gripper_command(node, gripper_action_name, gripper_closed_pos, gripper_max_effort, action_timeout_s)) {
      return fail("Failed to close gripper.");
    }
  }

  // 5) Retreat.
  if (enable_arm) {
    RCLCPP_INFO(node->get_logger(), "Retreating...");
    if (!plan_and_execute_pose(arm, pregrasp, execute, node->get_logger())) {
      return fail("Failed to retreat.");
    }
  }

  RCLCPP_INFO(node->get_logger(), "Pick sequence completed.");

  exec.cancel();
  rclcpp::shutdown();
  if (spin_thread.joinable()) spin_thread.join();
  return 0;
}
