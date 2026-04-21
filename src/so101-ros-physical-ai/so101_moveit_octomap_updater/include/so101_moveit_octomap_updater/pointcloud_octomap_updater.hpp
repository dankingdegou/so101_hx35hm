#pragma once

#include <moveit/occupancy_map_monitor/occupancy_map_updater.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <mutex>
#include <string>

namespace so101_moveit_octomap_updater
{

class PointCloudOctomapUpdater final : public occupancy_map_monitor::OccupancyMapUpdater
{
public:
  PointCloudOctomapUpdater();
  ~PointCloudOctomapUpdater() override;

  bool setParams(const std::string& name_space) override;
  bool initialize(const rclcpp::Node::SharedPtr& node) override;

  void start() override;
  void stop() override;

  occupancy_map_monitor::ShapeHandle excludeShape(const shapes::ShapeConstPtr& shape) override;
  void forgetShape(occupancy_map_monitor::ShapeHandle handle) override;

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  bool readParamOrDeclare_(const rclcpp::Node::SharedPtr& node, const std::string& key, std::string& out,
                           const std::string& def);
  bool readParamOrDeclare_(const rclcpp::Node::SharedPtr& node, const std::string& key, double& out, double def);
  bool readParamOrDeclare_(const rclcpp::Node::SharedPtr& node, const std::string& key, int& out, int def);

  std::string ns_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  std::mutex mutex_;
  bool running_{false};

  // Parameters
  // Default to the so101_openni2_camera namespace used by bringup configs.
  std::string point_cloud_topic_{"/static_camera/depth_overhead/points"};
  double max_range_{2.5};
  int point_subsample_{1};
  double padding_offset_{0.02};
  double padding_scale_{1.0};
  std::string filtered_cloud_topic_{""};  // ignored in this minimal implementation
};

}  // namespace so101_moveit_octomap_updater
