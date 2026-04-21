#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include <OpenNI.h>

namespace
{

#ifndef SO101_OPENNI2_DEFAULT_REDIST
#define SO101_OPENNI2_DEFAULT_REDIST "/usr/lib/x86_64-linux-gnu"
#endif

inline std::string openni_status_to_string(openni::Status rc)
{
  return std::string(openni::OpenNI::getExtendedError()) + " (rc=" + std::to_string(static_cast<int>(rc)) +
         ")";
}

inline sensor_msgs::msg::CameraInfo make_pinhole_info(
  int width, int height, float hfov_rad, float vfov_rad, const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  // Fallback camera model derived from field-of-view.
  // This is good enough for pointcloud generation and many downstream users,
  // but for high precision you should calibrate and publish a real camera_info.
  const double fx = static_cast<double>(width) / (2.0 * std::tan(static_cast<double>(hfov_rad) / 2.0));
  const double fy = static_cast<double>(height) / (2.0 * std::tan(static_cast<double>(vfov_rad) / 2.0));
  const double cx = (static_cast<double>(width) - 1.0) / 2.0;
  const double cy = (static_cast<double>(height) - 1.0) / 2.0;

  sensor_msgs::msg::CameraInfo info;
  info.header.stamp = stamp;
  info.header.frame_id = frame_id;
  info.width = static_cast<uint32_t>(width);
  info.height = static_cast<uint32_t>(height);
  info.distortion_model = "plumb_bob";
  info.d.assign(5, 0.0);

  info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  return info;
}

}  // namespace

class OpenNI2CameraNode final : public rclcpp::Node
{
public:
  OpenNI2CameraNode()
  : rclcpp::Node("openni2_camera_node")
  {
    // Make OpenNI2 driver discovery robust.
    // We default OPENNI2_REDIST to the directory that contains the OpenNI2 redistributable
    // (OpenNI.ini + OpenNI2/Drivers) that matches the OpenNI2 library we link against.
    if (std::getenv("OPENNI2_REDIST") == nullptr) {
      setenv("OPENNI2_REDIST", SO101_OPENNI2_DEFAULT_REDIST, 0);
    }

    device_uri_ = this->declare_parameter<std::string>("device_uri", "");
    frame_id_ = this->declare_parameter<std::string>("frame_id", "camera_depth");

    publish_depth_ = this->declare_parameter<bool>("publish_depth", true);
    publish_camera_info_ = this->declare_parameter<bool>("publish_camera_info", true);
    publish_pointcloud_ = this->declare_parameter<bool>("publish_pointcloud", true);

    depth_width_ = this->declare_parameter<int>("depth_width", 640);
    depth_height_ = this->declare_parameter<int>("depth_height", 480);
    depth_fps_ = this->declare_parameter<int>("depth_fps", 30);

    // Pointcloud throttling
    pointcloud_hz_ = this->declare_parameter<double>("pointcloud_hz", 10.0);
    pointcloud_stride_ = this->declare_parameter<int>("pointcloud_stride", 4);
    pointcloud_max_range_m_ = this->declare_parameter<double>("pointcloud_max_range_m", 2.5);
    pointcloud_min_range_m_ = this->declare_parameter<double>("pointcloud_min_range_m", 0.15);

    if (pointcloud_stride_ < 1) {
      RCLCPP_WARN(get_logger(), "pointcloud_stride < 1, forcing to 1");
      pointcloud_stride_ = 1;
    }
    if (pointcloud_hz_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "pointcloud_hz <= 0, disabling pointcloud publishing");
      publish_pointcloud_ = false;
    }

    if (publish_depth_) {
      depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("depth/image_raw", rclcpp::SensorDataQoS());
    }
    if (publish_camera_info_) {
      info_pub_ =
        this->create_publisher<sensor_msgs::msg::CameraInfo>("depth/camera_info", rclcpp::SensorDataQoS());
    }
    if (publish_pointcloud_) {
      cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("points", rclcpp::SensorDataQoS());
    }

    openni_init_();
    start_depth_stream_();

    // Use a background thread: OpenNI2 frame reads are typically blocking.
    // This keeps the ROS executor responsive (parameters/services, other nodes in the same process, etc).
    running_.store(true);
    worker_ = std::thread([this]() { this->run_loop_(); });

    if (publish_pointcloud_) {
      cloud_running_.store(true);
      cloud_worker_ = std::thread([this]() { this->cloud_loop_(); });
    }
  }

  ~OpenNI2CameraNode() override
  {
    try {
      running_.store(false);
      if (worker_.joinable()) {
        worker_.join();
      }
      cloud_running_.store(false);
      cloud_cv_.notify_all();
      if (cloud_worker_.joinable()) {
        cloud_worker_.join();
      }
      depth_stream_.stop();
      depth_stream_.destroy();
      device_.close();
      openni::OpenNI::shutdown();
    } catch (...) {
      // Best-effort cleanup
    }
  }

private:
  void openni_init_()
  {
    const openni::Status rc = openni::OpenNI::initialize();
    if (rc != openni::STATUS_OK) {
      throw std::runtime_error("OpenNI::initialize failed: " + openni_status_to_string(rc));
    }

    if (device_uri_.empty()) {
      RCLCPP_INFO(get_logger(), "Opening OpenNI2 device: ANY_DEVICE");
      const openni::Status rc_open = device_.open(openni::ANY_DEVICE);
      if (rc_open != openni::STATUS_OK) {
        throw std::runtime_error("Device::open(ANY_DEVICE) failed: " + openni_status_to_string(rc_open));
      }
    } else {
      RCLCPP_INFO(get_logger(), "Opening OpenNI2 device: '%s'", device_uri_.c_str());
      const openni::Status rc_open = device_.open(device_uri_.c_str());
      if (rc_open != openni::STATUS_OK) {
        throw std::runtime_error("Device::open(uri) failed: " + openni_status_to_string(rc_open));
      }
    }
  }

  void start_depth_stream_()
  {
    if (!device_.hasSensor(openni::SENSOR_DEPTH)) {
      throw std::runtime_error("Device does not report SENSOR_DEPTH");
    }

    openni::Status rc = depth_stream_.create(device_, openni::SENSOR_DEPTH);
    if (rc != openni::STATUS_OK) {
      throw std::runtime_error("Depth stream create failed: " + openni_status_to_string(rc));
    }

    // Configure video mode (resolution/fps/pixel format).
    openni::VideoMode vm = depth_stream_.getVideoMode();
    vm.setResolution(depth_width_, depth_height_);
    vm.setFps(depth_fps_);
    vm.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);  // uint16 mm
    rc = depth_stream_.setVideoMode(vm);
    if (rc != openni::STATUS_OK) {
      RCLCPP_WARN(get_logger(), "setVideoMode failed, continuing with driver default: %s",
        openni_status_to_string(rc).c_str());
    }

    rc = depth_stream_.start();
    if (rc != openni::STATUS_OK) {
      throw std::runtime_error("Depth stream start failed: " + openni_status_to_string(rc));
    }

    depth_hfov_rad_ = depth_stream_.getHorizontalFieldOfView();
    depth_vfov_rad_ = depth_stream_.getVerticalFieldOfView();

    RCLCPP_INFO(
      get_logger(),
      "OpenNI2 depth stream started: %dx%d@%d, hfov=%.3f rad, vfov=%.3f rad",
      depth_stream_.getVideoMode().getResolutionX(), depth_stream_.getVideoMode().getResolutionY(),
      depth_stream_.getVideoMode().getFps(), depth_hfov_rad_, depth_vfov_rad_);
  }

  void poll_once_()
  {
    if (!publish_depth_ && !publish_camera_info_ && !publish_pointcloud_) {
      return;
    }

    openni::VideoFrameRef frame;
    const openni::Status rc = depth_stream_.readFrame(&frame);
    if (rc != openni::STATUS_OK) {
      // Avoid spamming logs; OpenNI2 can occasionally return temporary read errors.
      RCLCPP_DEBUG(get_logger(), "readFrame failed: %s", openni_status_to_string(rc).c_str());
      return;
    }
    if (!frame.isValid()) {
      return;
    }

    const int width = frame.getWidth();
    const int height = frame.getHeight();
    const auto stamp = this->now();

    const uint16_t * depth_mm = static_cast<const uint16_t *>(frame.getData());
    if (depth_mm == nullptr) {
      return;
    }

    // If the pointcloud thread is requesting a frame, copy only on-demand.
    if (publish_pointcloud_ && cloud_request_.exchange(false)) {
      std::lock_guard<std::mutex> lk(cloud_mtx_);
      cloud_width_ = width;
      cloud_height_ = height;
      cloud_stamp_ = stamp;
      cloud_depth_mm_.assign(depth_mm, depth_mm + (static_cast<size_t>(width) * static_cast<size_t>(height)));
      cloud_has_frame_ = true;
      cloud_cv_.notify_one();
    }

    if (publish_depth_) {
      sensor_msgs::msg::Image msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = frame_id_;
      msg.height = static_cast<uint32_t>(height);
      msg.width = static_cast<uint32_t>(width);
      msg.encoding = "16UC1";
      msg.is_bigendian = false;
      msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(width * sizeof(uint16_t));
      msg.data.resize(static_cast<size_t>(msg.step) * msg.height);
      std::memcpy(msg.data.data(), depth_mm, msg.data.size());
      depth_pub_->publish(std::move(msg));
    }

    if (publish_camera_info_) {
      info_pub_->publish(make_pinhole_info(width, height, depth_hfov_rad_, depth_vfov_rad_, frame_id_, stamp));
    }
  }

  void run_loop_()
  {
    openni::VideoStream * streams[] = {&depth_stream_};
    while (rclcpp::ok() && running_.load()) {
      int changed_idx = 0;
      // Timeout keeps shutdown responsive without busy waiting.
      const openni::Status rc_wait = openni::OpenNI::waitForAnyStream(streams, 1, &changed_idx, 200);
      if (!running_.load()) {
        break;
      }
      if (rc_wait != openni::STATUS_OK) {
        continue;
      }
      // In practice we only have one stream; changed_idx is ignored.
      poll_once_();
    }
  }

  void cloud_loop_()
  {
    const rclcpp::Duration pc_period = rclcpp::Duration::from_seconds(1.0 / pointcloud_hz_);
    rclcpp::Time next = this->now();

    while (rclcpp::ok() && cloud_running_.load()) {
      // Sleep until next tick (best-effort).
      const rclcpp::Time now = this->now();
      if (now < next) {
        const auto sleep_ns = (next - now).nanoseconds();
        std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_ns));
      } else {
        next = now;
      }
      next = next + pc_period;

      // Request a frame from the depth thread and wait for it.
      cloud_request_.store(true);

      std::unique_lock<std::mutex> lk(cloud_mtx_);
      cloud_cv_.wait_for(lk, std::chrono::milliseconds(400), [this]() { return !cloud_running_.load() || cloud_has_frame_; });
      if (!cloud_running_.load()) {
        break;
      }
      if (!cloud_has_frame_) {
        continue;
      }

      // Copy out to minimize time holding the lock.
      const int w = cloud_width_;
      const int h = cloud_height_;
      const rclcpp::Time stamp = cloud_stamp_;
      std::vector<uint16_t> depth = cloud_depth_mm_;
      cloud_has_frame_ = false;
      lk.unlock();

      publish_cloud_(depth.data(), w, h, stamp);
      last_cloud_stamp_ = stamp;
    }
  }

  void publish_cloud_(const uint16_t * depth_mm, int width, int height, const rclcpp::Time & stamp)
  {
    // Build an XYZ-only point cloud in the camera frame.
    // Use OpenNI2's own coordinate converter (device-calibrated) to avoid guessing intrinsics.

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = frame_id_;
    cloud.height = 1;

    const int stride = pointcloud_stride_;
    const int sampled_w = (width + stride - 1) / stride;
    const int sampled_h = (height + stride - 1) / stride;
    const size_t max_points = static_cast<size_t>(sampled_w) * static_cast<size_t>(sampled_h);

    cloud.width = static_cast<uint32_t>(max_points);
    cloud.is_bigendian = false;
    cloud.is_dense = false;  // we will insert NaNs for invalid ranges

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(max_points);

    sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");

    const float nan = std::numeric_limits<float>::quiet_NaN();

    size_t idx = 0;
    for (int v = 0; v < height; v += stride) {
      for (int u = 0; u < width; u += stride) {
        const uint16_t z_mm = depth_mm[v * width + u];
        float x = nan, y = nan, z = nan;

        if (z_mm != 0) {
          float wx_mm = 0.0f, wy_mm = 0.0f, wz_mm = 0.0f;
          // OpenNI convention: X right, Y up, Z towards the camera? (depends on driver).
          // For collision avoidance, consistency matters more than absolute convention as long as TF is correct.
          openni::CoordinateConverter::convertDepthToWorld(depth_stream_, u, v, z_mm, &wx_mm, &wy_mm, &wz_mm);
          const double wz_m = static_cast<double>(wz_mm) / 1000.0;
          if (wz_m >= pointcloud_min_range_m_ && wz_m <= pointcloud_max_range_m_) {
            x = static_cast<float>(static_cast<double>(wx_mm) / 1000.0);
            y = static_cast<float>(static_cast<double>(wy_mm) / 1000.0);
            z = static_cast<float>(wz_m);
          }
        }

        *it_x = x;
        *it_y = y;
        *it_z = z;
        ++it_x;
        ++it_y;
        ++it_z;
        ++idx;
      }
    }

    // If width/height are not exact multiples of stride, we may have unused points at the tail.
    // Shrink to the actual number we iterated.
    if (idx < max_points) {
      cloud.width = static_cast<uint32_t>(idx);
      modifier.resize(idx);
    }

    cloud_pub_->publish(std::move(cloud));
  }

private:
  std::string device_uri_;
  std::string frame_id_;

  bool publish_depth_{true};
  bool publish_camera_info_{true};
  bool publish_pointcloud_{true};

  int depth_width_{640};
  int depth_height_{480};
  int depth_fps_{30};

  double pointcloud_hz_{10.0};
  int pointcloud_stride_{4};
  double pointcloud_max_range_m_{2.5};
  double pointcloud_min_range_m_{0.15};

  float depth_hfov_rad_{0.0f};
  float depth_vfov_rad_{0.0f};

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

  rclcpp::Time last_cloud_stamp_{0, 0, RCL_ROS_TIME};

  std::atomic<bool> running_{false};
  std::thread worker_;

  openni::Device device_;
  openni::VideoStream depth_stream_;

  // Pointcloud worker (decoupled from depth read loop to reduce depth FPS jitter).
  std::atomic<bool> cloud_running_{false};
  std::atomic<bool> cloud_request_{false};
  std::thread cloud_worker_;
  std::mutex cloud_mtx_;
  std::condition_variable cloud_cv_;
  bool cloud_has_frame_{false};
  int cloud_width_{0};
  int cloud_height_{0};
  rclcpp::Time cloud_stamp_{0, 0, RCL_ROS_TIME};
  std::vector<uint16_t> cloud_depth_mm_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<OpenNI2CameraNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    // If OpenNI2 init fails, we still want a readable error in ros2 launch output.
    fprintf(stderr, "OpenNI2CameraNode fatal error: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
