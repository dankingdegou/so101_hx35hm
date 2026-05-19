#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <chrono>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

class FollowerCommandRelay : public rclcpp::Node
{
public:
  FollowerCommandRelay() : Node("leader_follower_teleop") {
    RCLCPP_INFO(get_logger(), "Initializing FollowerCommandRelay...");

    // Parameters
    // Arm output mode:
    //  - "joint_trajectory" => JointTrajectoryController topic
    //  - "forward_position" => ForwardController commands topic
    arm_mode_ = this->declare_parameter<std::string>("arm_mode", "joint_trajectory");

    leader_topic_ = declare_parameter<std::string>("leader_topic", "/leader/joint_states");
    follower_topic_ = declare_parameter<std::string>("follower_topic", "/follower/joint_states");
    follower_jtc_topic_ = declare_parameter<std::string>(
        "jtc_topic", "/follower/trajectory_controller/joint_trajectory");
    follower_fwd_topic_ =
        declare_parameter<std::string>("fwd_topic", "/follower/forward_controller/commands");
    mapping_mode_ = declare_parameter<std::string>("mapping_mode", "absolute");

    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);
    stale_timeout_s_ = declare_parameter<double>("stale_timeout_s", 0.25);
    point_dt_s_ = declare_parameter<double>("point_dt_s", 0.06);
    filter_mode_ = declare_parameter<std::string>("filter_mode", "lpf");
    lpf_alpha_ = declare_parameter<double>("lpf_alpha", 1.0);
    alpha_beta_alpha_ = declare_parameter<double>("alpha_beta_alpha", 0.75);
    alpha_beta_beta_ = declare_parameter<double>("alpha_beta_beta", 0.08);
    prediction_dt_s_ = declare_parameter<double>("prediction_dt_s", 0.02);

    arm_joints_ = declare_parameter<std::vector<std::string>>(
        "arm_joints", std::vector<std::string>{"shoulder_pan", "shoulder_lift", "elbow_flex",
                                               "wrist_flex", "wrist_roll", "gripper"});
    joint_scales_ = declare_parameter<std::vector<double>>(
        "joint_scales", std::vector<double>(arm_joints_.size(), 1.0));
    joint_offsets_ = declare_parameter<std::vector<double>>(
        "joint_offsets", std::vector<double>(arm_joints_.size(), 0.0));

    if (joint_scales_.size() != arm_joints_.size()) {
      RCLCPP_WARN(
          get_logger(),
          "joint_scales length (%zu) does not match arm_joints (%zu); falling back to all 1.0",
          joint_scales_.size(), arm_joints_.size());
      joint_scales_.assign(arm_joints_.size(), 1.0);
    }
    if (joint_offsets_.size() != arm_joints_.size()) {
      RCLCPP_WARN(
          get_logger(),
          "joint_offsets length (%zu) does not match arm_joints (%zu); falling back to all 0.0",
          joint_offsets_.size(), arm_joints_.size());
      joint_offsets_.assign(arm_joints_.size(), 0.0);
    }

    filtered_.assign(arm_joints_.size(), 0.0);

    RCLCPP_INFO(get_logger(), "Leader: %s", leader_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Follower state: %s", follower_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Follower JTC: %s", follower_jtc_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Mapping mode: %s", mapping_mode_.c_str());
    RCLCPP_INFO(get_logger(), "Rate: %.1f Hz, Arm joints: %zu", publish_rate_hz_,
                arm_joints_.size());

    // ROS interfaces
    leader_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        leader_topic_, rclcpp::SensorDataQoS(),
        std::bind(&FollowerCommandRelay::joint_state_callback, this, std::placeholders::_1));
    follower_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        follower_topic_, rclcpp::SensorDataQoS(),
        std::bind(&FollowerCommandRelay::follower_state_callback, this, std::placeholders::_1));

    trajectory_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
        follower_jtc_topic_, rclcpp::QoS(10).reliable());
    auto forward_command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    forward_pub_ =
        create_publisher<std_msgs::msg::Float64MultiArray>(follower_fwd_topic_, forward_command_qos);

    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate_hz_),
                               std::bind(&FollowerCommandRelay::control_loop, this));

    raw_arm_.resize(arm_joints_.size(), 0.0);
    ab_position_.resize(arm_joints_.size(), 0.0);
    ab_velocity_.resize(arm_joints_.size(), 0.0);
    leader_home_.resize(arm_joints_.size(), 0.0);
    follower_home_.resize(arm_joints_.size(), 0.0);
    latest_leader_.resize(arm_joints_.size(), 0.0);
    latest_follower_.resize(arm_joints_.size(), 0.0);

    RCLCPP_INFO(get_logger(), "FollowerCommandRelay initialized.");
  }

private:
  // Parameters
  std::string arm_mode_;
  std::string leader_topic_;
  std::string follower_topic_;
  std::string follower_jtc_topic_;
  std::string follower_fwd_topic_;
  std::string mapping_mode_;
  std::string filter_mode_;
  double publish_rate_hz_{50.0};
  double stale_timeout_s_{0.25};
  double point_dt_s_{0.02};
  double lpf_alpha_{1.0}; // 1.0 = no filtering
  double alpha_beta_alpha_{0.75};
  double alpha_beta_beta_{0.08};
  double prediction_dt_s_{0.02};
  bool have_filtered_{false};
  bool have_alpha_beta_{false};
  std::vector<std::string> arm_joints_;
  std::vector<double> filtered_;
  std::vector<double> ab_position_;
  std::vector<double> ab_velocity_;
  std::vector<double> joint_scales_;
  std::vector<double> joint_offsets_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr leader_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr follower_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr forward_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  bool initialized_{false};
  bool follower_initialized_{false};
  bool have_leader_home_{false};
  bool have_follower_home_{false};
  bool relative_ready_logged_{false};
  std::vector<int> arm_idx_;
  std::vector<int> follower_idx_;
  std::vector<double> raw_arm_;
  std::vector<double> leader_home_;
  std::vector<double> follower_home_;
  std::vector<double> latest_leader_;
  std::vector<double> latest_follower_;
  rclcpp::Time last_leader_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_filter_stamp_{0, 0, RCL_ROS_TIME};

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {

    if (!initialized_ && !initialize_indices(*msg)) return;

    last_leader_stamp_ = this->now();

    // Cache leader state and targets.
    for (size_t i = 0; i < arm_joints_.size(); ++i) {
      latest_leader_[i] = msg->position[arm_idx_[i]];
    }

    if (mapping_mode_ == "relative") {
      if (!have_leader_home_) {
        leader_home_ = latest_leader_;
        have_leader_home_ = true;
        RCLCPP_INFO(get_logger(), "Captured leader relative baseline.");
      }
      update_relative_targets();
      return;
    }

    for (size_t i = 0; i < arm_joints_.size(); ++i) {
      raw_arm_[i] = latest_leader_[i] * joint_scales_[i] + joint_offsets_[i];
    }
  }

  bool initialize_indices(const sensor_msgs::msg::JointState &msg) {
    // Build name -> index map
    std::unordered_map<std::string, int> idx_map;
    idx_map.reserve(msg.name.size());
    for (size_t i = 0; i < msg.name.size(); i++)
      idx_map[msg.name[i]] = static_cast<int>(i);

    // Arm indexes
    arm_idx_.assign(arm_joints_.size(), -1);
    for (size_t i = 0; i < arm_joints_.size(); ++i) {
      auto it = idx_map.find(arm_joints_[i]);
      if (it == idx_map.end()) {
        RCLCPP_ERROR(this->get_logger(), "Leader arm joint '%s' not found", arm_joints_[i].c_str());
        return false;
      }
      arm_idx_[i] = it->second;
    }

    initialized_ = true;
    RCLCPP_INFO(get_logger(), "Initialized: %zu arm joints", arm_joints_.size());
    return true;
  }

  void follower_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!follower_initialized_ && !initialize_follower_indices(*msg)) return;

    for (size_t i = 0; i < arm_joints_.size(); ++i) {
      latest_follower_[i] = msg->position[follower_idx_[i]];
    }

    if (mapping_mode_ == "relative" && !have_follower_home_) {
      follower_home_ = latest_follower_;
      raw_arm_ = follower_home_;
      have_follower_home_ = true;
      RCLCPP_INFO(get_logger(), "Captured follower relative baseline.");
      update_relative_targets();
    }
  }

  bool initialize_follower_indices(const sensor_msgs::msg::JointState &msg) {
    std::unordered_map<std::string, int> idx_map;
    idx_map.reserve(msg.name.size());
    for (size_t i = 0; i < msg.name.size(); i++)
      idx_map[msg.name[i]] = static_cast<int>(i);

    follower_idx_.assign(arm_joints_.size(), -1);
    for (size_t i = 0; i < arm_joints_.size(); ++i) {
      auto it = idx_map.find(arm_joints_[i]);
      if (it == idx_map.end()) {
        RCLCPP_ERROR(
            this->get_logger(), "Follower arm joint '%s' not found", arm_joints_[i].c_str());
        return false;
      }
      follower_idx_[i] = it->second;
    }

    follower_initialized_ = true;
    RCLCPP_INFO(get_logger(), "Initialized follower baseline reader: %zu arm joints",
                arm_joints_.size());
    return true;
  }

  void update_relative_targets() {
    if (!have_leader_home_ || !have_follower_home_) return;

    for (size_t i = 0; i < arm_joints_.size(); ++i) {
      raw_arm_[i] =
          follower_home_[i] + (latest_leader_[i] - leader_home_[i]) * joint_scales_[i] +
          joint_offsets_[i];
    }

    if (!relative_ready_logged_) {
      RCLCPP_INFO(get_logger(), "Relative teleop ready: follower targets follow leader deltas.");
      relative_ready_logged_ = true;
    }
  }

  void control_loop() {

    if (!initialized_) return;
    if (mapping_mode_ == "relative" && (!have_leader_home_ || !have_follower_home_)) return;

    const auto now = this->now();

    // Leader data stale: do nothing (holds last command on follower)
    if ((now - last_leader_stamp_).seconds() > stale_timeout_s_) return;

    if (filter_mode_ == "alpha_beta") {
      update_alpha_beta_filter(now);
    } else if (!have_filtered_ || lpf_alpha_ >= 0.999) {
        filtered_ = raw_arm_;
        have_filtered_ = true;
    } else {
        for (size_t i = 0; i < raw_arm_.size(); ++i) {
          filtered_[i] = lpf_alpha_ * raw_arm_[i] + (1.0 - lpf_alpha_) * filtered_[i];
        }
    }

    publish_arm(now, filtered_);
  }

  void update_alpha_beta_filter(const rclcpp::Time &now) {
    if (!have_alpha_beta_) {
      ab_position_ = raw_arm_;
      ab_velocity_.assign(raw_arm_.size(), 0.0);
      filtered_ = raw_arm_;
      last_filter_stamp_ = now;
      have_alpha_beta_ = true;
      have_filtered_ = true;
      return;
    }

    double dt = (now - last_filter_stamp_).seconds();
    if (dt <= 0.0 || dt > stale_timeout_s_) {
      dt = 1.0 / publish_rate_hz_;
    }
    last_filter_stamp_ = now;

    const double alpha = clamp01(alpha_beta_alpha_);
    const double beta = clamp01(alpha_beta_beta_);
    const double predict_dt = prediction_dt_s_ > 0.0 ? prediction_dt_s_ : 0.0;

    for (size_t i = 0; i < raw_arm_.size(); ++i) {
      const double predicted = ab_position_[i] + ab_velocity_[i] * dt;
      const double residual = raw_arm_[i] - predicted;
      ab_position_[i] = predicted + alpha * residual;
      ab_velocity_[i] = ab_velocity_[i] + (beta / dt) * residual;
      filtered_[i] = ab_position_[i] + ab_velocity_[i] * predict_dt;
    }
  }

  double clamp01(double value) const {
    if (value < 0.0) return 0.0;
    if (value > 1.0) return 1.0;
    return value;
  }

  void publish_arm(const rclcpp::Time &time, const std::vector<double> &positions) {
    if (arm_mode_ == "joint_trajectory") {
      trajectory_msgs::msg::JointTrajectory jt;
      jt.header.stamp = time;
      jt.joint_names = arm_joints_;

      trajectory_msgs::msg::JointTrajectoryPoint pt;
      pt.positions = positions;

      const int sec = static_cast<int>(point_dt_s_);
      const int nsec = static_cast<int>((point_dt_s_ - sec) * 1e9);
      pt.time_from_start.sec = sec;
      pt.time_from_start.nanosec = nsec;
      jt.points.push_back(pt);
      trajectory_pub_->publish(jt);
    } else {
      std_msgs::msg::Float64MultiArray cmd;
      cmd.data = positions;
      forward_pub_->publish(cmd);
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FollowerCommandRelay>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
