#include "so101_moveit_octomap_updater/pointcloud_octomap_updater.hpp"

#include <moveit/occupancy_map_monitor/occupancy_map_monitor.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <octomap/octomap.h>

#include <pluginlib/class_list_macros.hpp>

#include <cmath>
#include <limits>

namespace so101_moveit_octomap_updater
{

PointCloudOctomapUpdater::PointCloudOctomapUpdater()
  : occupancy_map_monitor::OccupancyMapUpdater("pointcloud")
{
}

PointCloudOctomapUpdater::~PointCloudOctomapUpdater()
{
  stop();
}

bool PointCloudOctomapUpdater::setParams(const std::string& name_space)
{
  ns_ = name_space;
  return true;
}

bool PointCloudOctomapUpdater::readParamOrDeclare_(
  const rclcpp::Node::SharedPtr& node, const std::string& key, std::string& out, const std::string& def)
{
  if (!node->has_parameter(key)) {
    node->declare_parameter<std::string>(key, def);
  }
  out = node->get_parameter(key).as_string();
  return true;
}

bool PointCloudOctomapUpdater::readParamOrDeclare_(
  const rclcpp::Node::SharedPtr& node, const std::string& key, double& out, double def)
{
  if (!node->has_parameter(key)) {
    node->declare_parameter<double>(key, def);
  }
  out = node->get_parameter(key).as_double();
  return true;
}

bool PointCloudOctomapUpdater::readParamOrDeclare_(
  const rclcpp::Node::SharedPtr& node, const std::string& key, int& out, int def)
{
  if (!node->has_parameter(key)) {
    node->declare_parameter<int>(key, def);
  }
  out = node->get_parameter(key).as_int();
  return true;
}

bool PointCloudOctomapUpdater::initialize(const rclcpp::Node::SharedPtr& node)
{
  node_ = node;

  // Occupancy map tree comes from the monitor.
  if (!monitor_) {
    RCLCPP_ERROR(node_->get_logger(), "PointCloudOctomapUpdater initialize(): monitor_ is null");
    return false;
  }
  tree_ = monitor_->getOcTreePtr();
  if (!tree_) {
    RCLCPP_ERROR(node_->get_logger(), "PointCloudOctomapUpdater initialize(): tree_ is null");
    return false;
  }

  // Read plugin params under "<ns_>.*"
  // These are set by so101_moveit_config/config/octomap_pointcloud.yaml
  // Some MoveIt builds pass an empty namespace to setParams(); in that case,
  // try to infer the intended namespace from the `sensors` string array.
  if (ns_.empty() && node_->has_parameter("sensors")) {
    try {
      const auto sensors = node_->get_parameter("sensors").as_string_array();
      if (!sensors.empty() && !sensors[0].empty()) {
        ns_ = sensors[0];
      }
    } catch (...) {
      // ignore
    }
  }

  const std::string prefix = ns_.empty() ? "" : (ns_ + ".");

  readParamOrDeclare_(node_, prefix + "point_cloud_topic", point_cloud_topic_, point_cloud_topic_);
  readParamOrDeclare_(node_, prefix + "max_range", max_range_, max_range_);
  readParamOrDeclare_(node_, prefix + "point_subsample", point_subsample_, point_subsample_);
  readParamOrDeclare_(node_, prefix + "padding_offset", padding_offset_, padding_offset_);
  readParamOrDeclare_(node_, prefix + "padding_scale", padding_scale_, padding_scale_);
  readParamOrDeclare_(node_, prefix + "filtered_cloud_topic", filtered_cloud_topic_, filtered_cloud_topic_);

  if (point_subsample_ < 1) {
    point_subsample_ = 1;
  }
  if (max_range_ <= 0.0) {
    max_range_ = 2.5;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "PointCloudOctomapUpdater initialized: ns='%s' topic='%s' max_range=%.2f subsample=%d",
    ns_.c_str(), point_cloud_topic_.c_str(), max_range_, point_subsample_);

  return true;
}

void PointCloudOctomapUpdater::start()
{
  std::lock_guard<std::mutex> lk(mutex_);
  if (running_ || !node_) {
    return;
  }

  running_ = true;
  sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    point_cloud_topic_, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { this->cloudCallback(msg); });
}

void PointCloudOctomapUpdater::stop()
{
  std::lock_guard<std::mutex> lk(mutex_);
  running_ = false;
  sub_.reset();
}

occupancy_map_monitor::ShapeHandle PointCloudOctomapUpdater::excludeShape(const shapes::ShapeConstPtr& /*shape*/)
{
  // Minimal implementation: no self-filtering.
  // Returning a non-zero handle allows MoveIt to call forgetShape(handle) later.
  static occupancy_map_monitor::ShapeHandle next = 1;
  return next++;
}

void PointCloudOctomapUpdater::forgetShape(occupancy_map_monitor::ShapeHandle /*handle*/)
{
  // Minimal implementation: no-op.
}

void PointCloudOctomapUpdater::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Guard against concurrent callbacks (move_group typically uses a multi-threaded executor).
  std::lock_guard<std::mutex> lk(mutex_);
  if (!running_ || !monitor_ || !tree_ || !msg) return;

  const std::string target_frame = monitor_->getMapFrame();  // usually "world"
  if (target_frame.empty()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "octomap_frame (map frame) is empty");
    return;
  }

  sensor_msgs::msg::PointCloud2 cloud_tf;
  try {
    const auto tf = monitor_->getTFClient()->lookupTransform(
      target_frame, msg->header.frame_id, tf2::TimePointZero);
    tf2::doTransform(*msg, cloud_tf, tf);
  } catch (const std::exception& e) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "TF transform failed (%s -> %s): %s", msg->header.frame_id.c_str(), target_frame.c_str(), e.what());
    return;
  }

  // Rebuild the octomap each update (simple + stable for tabletop use).
  // This does not model free space; it only marks occupied cells.
  {
    auto lock = tree_->writing();
    tree_->clear();

    const float max_r2 = static_cast<float>(max_range_ * max_range_);

    sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud_tf, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(cloud_tf, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(cloud_tf, "z");

    int idx = 0;
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++idx) {
      if ((idx % point_subsample_) != 0) {
        continue;
      }
      const float x = *it_x;
      const float y = *it_y;
      const float z = *it_z;
      if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
        continue;
      }

      const float r2 = x * x + y * y + z * z;
      if (r2 > max_r2) {
        continue;
      }

      // Apply a simple padding: expand occupied space slightly.
      // MoveIt's original updater uses padding_offset/scale during collision checking; we approximate here.
      const float px = x * static_cast<float>(padding_scale_);
      const float py = y * static_cast<float>(padding_scale_);
      const float pz = z * static_cast<float>(padding_scale_);
      tree_->updateNode(octomap::point3d(px, py, pz), true);
    }

    tree_->updateInnerOccupancy();
  }

  // Important: triggerUpdateCallback() can call back into code that also tries to lock the tree.
  // Calling it while holding the tree write lock can throw std::system_error (EDEADLK) on some builds.
  tree_->triggerUpdateCallback();
}

}  // namespace so101_moveit_octomap_updater

PLUGINLIB_EXPORT_CLASS(
  so101_moveit_octomap_updater::PointCloudOctomapUpdater, occupancy_map_monitor::OccupancyMapUpdater)
