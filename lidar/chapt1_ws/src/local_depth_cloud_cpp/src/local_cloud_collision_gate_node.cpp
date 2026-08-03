#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

namespace
{
using sensor_msgs::msg::LaserScan;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;
using SteadyClock = std::chrono::steady_clock;

bool find_float32_offset(const PointCloud2 & cloud, const std::string & name, uint32_t & offset)
{
  for (const auto & field : cloud.fields) {
    if (field.name == name && field.datatype == PointField::FLOAT32 && field.count >= 1U) {
      offset = field.offset;
      return true;
    }
  }
  return false;
}

float read_float32(const uint8_t * data, uint32_t offset)
{
  float value = 0.0F;
  std::memcpy(&value, data + offset, sizeof(value));
  return value;
}
}  // namespace

class LocalCloudCollisionGate : public rclcpp::Node
{
public:
  LocalCloudCollisionGate()
  : Node("local_cloud_collision_gate")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/local_highres_cloud_v21");
    scan_topic_ = declare_parameter<std::string>(
      "scan_topic", "/scan_timed_v2_filtered");
    stop_topic_ = declare_parameter<std::string>(
      "stop_topic", "/local_cloud_collision_stop");
    status_topic_ = declare_parameter<std::string>(
      "status_topic", "/local_cloud_collision_status");

    front_x_min_ = declare_parameter<double>("x_min", 0.20);
    front_x_max_ = declare_parameter<double>("x_max", 0.62);
    front_half_width_ = declare_parameter<double>("half_width", 0.39);
    z_min_ = declare_parameter<double>("z_min", 0.02);
    z_max_ = declare_parameter<double>("z_max", 1.40);
    cloud_min_points_ = std::max(
      1, static_cast<int>(declare_parameter<int64_t>("min_points", 6)));
    approach_x_min_ = declare_parameter<double>("approach_x_min", 0.20);
    approach_x_max_ = declare_parameter<double>("approach_x_max", 1.20);
    approach_half_width_ = declare_parameter<double>("approach_half_width", 0.50);
    approach_min_points_ = std::max(
      1, static_cast<int>(declare_parameter<int64_t>("approach_min_points", 3)));

    rear_x_min_ = declare_parameter<double>("rear_x_min", -0.62);
    rear_x_max_ = declare_parameter<double>("rear_x_max", -0.20);
    rear_half_width_ = declare_parameter<double>("rear_half_width", 0.39);
    rotation_radius_ = declare_parameter<double>("rotation_radius", 0.52);
    scan_self_filter_half_length_ = std::max(
      0.0, declare_parameter<double>("scan_self_filter_half_length", 0.33));
    scan_self_filter_half_width_ = std::max(
      0.0, declare_parameter<double>("scan_self_filter_half_width", 0.33));
    scan_min_points_ = std::max(
      1, static_cast<int>(declare_parameter<int64_t>("scan_min_points", 2)));
    laser_x_ = declare_parameter<double>("laser_x", 0.20);
    laser_y_ = declare_parameter<double>("laser_y", 0.0);
    laser_yaw_ = declare_parameter<double>("laser_yaw", 0.0);
    scan_timeout_sec_ = std::max(
      0.05, declare_parameter<double>("scan_timeout_sec", 0.35));

    hold_sec_ = std::max(0.0, declare_parameter<double>("hold_sec", 0.25));
    sample_stride_ = std::max(
      1, static_cast<int>(declare_parameter<int64_t>("sample_stride", 1)));
    status_period_sec_ = std::max(
      0.2, declare_parameter<double>("status_period_sec", 1.0));

    stop_pub_ = create_publisher<std_msgs::msg::Bool>(stop_topic_, rclcpp::QoS(10));
    status_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(
      status_topic_, rclcpp::QoS(10));
    cloud_sub_ = create_subscription<PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&LocalCloudCollisionGate::on_cloud, this, std::placeholders::_1));
    scan_sub_ = create_subscription<LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS().keep_last(5),
      std::bind(&LocalCloudCollisionGate::on_scan, this, std::placeholders::_1));
    publish_timer_ = create_wall_timer(
      std::chrono::milliseconds(33),
      std::bind(&LocalCloudCollisionGate::publish_state, this));
    status_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(status_period_sec_)),
      std::bind(&LocalCloudCollisionGate::log_status, this));

    RCLCPP_INFO(
      get_logger(),
      "C++ collision gate: RGB-D front x=[%.2f,%.2f], 2D scan=%s "
      "approach=[%.2f,%.2f], rear=[%.2f,%.2f], rotation_radius=%.2f m, "
      "scan_self_filter=[+/-%.3f,+/-%.3f] m",
      front_x_min_, front_x_max_, scan_topic_.c_str(),
      approach_x_min_, approach_x_max_, rear_x_min_, rear_x_max_,
      rotation_radius_, scan_self_filter_half_length_,
      scan_self_filter_half_width_);
  }

private:
  SteadyClock::time_point hold_deadline() const
  {
    return SteadyClock::now() + std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(hold_sec_));
  }

  static bool held(const SteadyClock::time_point & deadline)
  {
    return SteadyClock::now() <= deadline;
  }

  bool scan_alive() const
  {
    return scan_count_ > 0U &&
           std::chrono::duration<double>(SteadyClock::now() - last_scan_time_).count() <=
           scan_timeout_sec_;
  }

  void on_cloud(const PointCloud2::SharedPtr cloud)
  {
    last_cloud_time_ = SteadyClock::now();
    ++cloud_count_;
    if (cloud->is_bigendian) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Big-endian PointCloud2 is unsupported by the collision gate");
      cloud_front_count_ = 0;
      cloud_nearest_mm_ = 9999;
      return;
    }

    uint32_t x_offset = 0U;
    uint32_t y_offset = 0U;
    uint32_t z_offset = 0U;
    if (!find_float32_offset(*cloud, "x", x_offset) ||
      !find_float32_offset(*cloud, "y", y_offset) ||
      !find_float32_offset(*cloud, "z", z_offset))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Collision cloud must contain FLOAT32 x/y/z fields");
      return;
    }
    const uint32_t required_size = std::max({x_offset, y_offset, z_offset}) + sizeof(float);
    if (cloud->point_step < required_size || cloud->row_step == 0U) {
      return;
    }

    int count = 0;
    int approach_count = 0;
    float nearest = std::numeric_limits<float>::infinity();
    float approach_nearest = std::numeric_limits<float>::infinity();
    int sampled_index = 0;
    for (uint32_t row = 0U; row < cloud->height; ++row) {
      const size_t row_base = static_cast<size_t>(row) * cloud->row_step;
      for (uint32_t col = 0U; col < cloud->width; ++col, ++sampled_index) {
        if (sampled_index % sample_stride_ != 0) {
          continue;
        }
        const size_t base = row_base + static_cast<size_t>(col) * cloud->point_step;
        if (base + required_size > cloud->data.size()) {
          break;
        }
        const uint8_t * point = cloud->data.data() + base;
        const float x = read_float32(point, x_offset);
        const float y = read_float32(point, y_offset);
        const float z = read_float32(point, z_offset);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }
        if (x >= front_x_min_ && x <= front_x_max_ &&
          std::abs(y) <= front_half_width_ && z >= z_min_ && z <= z_max_)
        {
          ++count;
          nearest = std::min(nearest, x);
        }
        if (x >= approach_x_min_ && x <= approach_x_max_ &&
          std::abs(y) <= approach_half_width_ && z >= z_min_ && z <= z_max_)
        {
          ++approach_count;
          approach_nearest = std::min(approach_nearest, x);
        }
      }
    }

    cloud_front_count_ = count;
    cloud_approach_count_ = approach_count;
    cloud_nearest_mm_ = std::isfinite(nearest) ?
      static_cast<int>(std::lround(nearest * 1000.0F)) : 9999;
    cloud_approach_nearest_mm_ = std::isfinite(approach_nearest) ?
      static_cast<int>(std::lround(approach_nearest * 1000.0F)) : 9999;
    if (count >= cloud_min_points_) {
      cloud_front_until_ = hold_deadline();
    }
    if (approach_count >= approach_min_points_) {
      cloud_approach_until_ = hold_deadline();
    }
    publish_state();
  }

  void on_scan(const LaserScan::SharedPtr scan)
  {
    last_scan_time_ = SteadyClock::now();
    ++scan_count_;
    int front_count = 0;
    int rear_count = 0;
    int rotation_count = 0;
    int approach_count = 0;
    int self_filtered_count = 0;
    float nearest_front = std::numeric_limits<float>::infinity();
    float nearest_approach = std::numeric_limits<float>::infinity();
    const double cos_yaw = std::cos(laser_yaw_);
    const double sin_yaw = std::sin(laser_yaw_);
    const double rotation_radius_sq = rotation_radius_ * rotation_radius_;

    for (size_t i = 0U; i < scan->ranges.size(); ++i) {
      const float range = scan->ranges[i];
      if (!std::isfinite(range) || range < scan->range_min || range > scan->range_max) {
        continue;
      }
      const double angle = scan->angle_min + static_cast<double>(i) * scan->angle_increment;
      const double laser_px = static_cast<double>(range) * std::cos(angle);
      const double laser_py = static_cast<double>(range) * std::sin(angle);
      const double x = laser_x_ + cos_yaw * laser_px - sin_yaw * laser_py;
      const double y = laser_y_ + sin_yaw * laser_px + cos_yaw * laser_py;

      // A real obstacle cannot occupy the chassis' physical footprint. Returns
      // in this box are top-plate/bracket/self reflections and must not make
      // every stop-and-turn or backup look permanently blocked.
      if (std::abs(x) <= scan_self_filter_half_length_ &&
        std::abs(y) <= scan_self_filter_half_width_)
      {
        ++self_filtered_count;
        continue;
      }

      if (x >= front_x_min_ && x <= front_x_max_ && std::abs(y) <= front_half_width_) {
        ++front_count;
        nearest_front = std::min(nearest_front, static_cast<float>(x));
      }
      if (x >= approach_x_min_ && x <= approach_x_max_ &&
        std::abs(y) <= approach_half_width_)
      {
        ++approach_count;
        nearest_approach = std::min(nearest_approach, static_cast<float>(x));
      }
      if (x >= rear_x_min_ && x <= rear_x_max_ && std::abs(y) <= rear_half_width_) {
        ++rear_count;
      }
      if (x * x + y * y <= rotation_radius_sq) {
        ++rotation_count;
      }
    }

    scan_front_count_ = front_count;
    scan_rear_count_ = rear_count;
    scan_rotation_count_ = rotation_count;
    scan_approach_count_ = approach_count;
    scan_self_filtered_count_ = self_filtered_count;
    scan_nearest_mm_ = std::isfinite(nearest_front) ?
      static_cast<int>(std::lround(nearest_front * 1000.0F)) : 9999;
    scan_approach_nearest_mm_ = std::isfinite(nearest_approach) ?
      static_cast<int>(std::lround(nearest_approach * 1000.0F)) : 9999;
    if (front_count >= scan_min_points_) {
      scan_front_until_ = hold_deadline();
    }
    if (rear_count >= scan_min_points_) {
      scan_rear_until_ = hold_deadline();
    }
    if (rotation_count >= scan_min_points_) {
      scan_rotation_until_ = hold_deadline();
    }
    if (approach_count >= scan_min_points_) {
      scan_approach_until_ = hold_deadline();
    }
    publish_state();
  }

  bool front_blocked() const
  {
    return held(cloud_front_until_) || held(scan_front_until_);
  }

  void publish_state()
  {
    const bool front = front_blocked();
    const bool rotation = held(scan_rotation_until_);
    const bool rear = held(scan_rear_until_);
    const int front_count = cloud_front_count_ + scan_front_count_;
    const int nearest_mm = std::min(cloud_nearest_mm_, scan_nearest_mm_);
    const bool approach =
      held(cloud_approach_until_) || held(scan_approach_until_);
    const int approach_count =
      cloud_approach_count_ + scan_approach_count_;
    const int approach_nearest_mm = approach ?
      std::min(cloud_approach_nearest_mm_, scan_approach_nearest_mm_) : 9999;

    std_msgs::msg::Bool stop;
    stop.data = front;
    stop_pub_->publish(stop);

    std_msgs::msg::Int32MultiArray status;
    // Keep the first three fields compatible with the previous RGB-D-only gate.
    status.data = {
      front ? 1 : 0, front_count, nearest_mm,
      rotation ? 1 : 0, rear ? 1 : 0,
      scan_rotation_count_, scan_rear_count_, scan_alive() ? 1 : 0,
      approach_count, approach_nearest_mm, scan_self_filtered_count_};
    status_pub_->publish(status);
  }

  void log_status()
  {
    const double cloud_age_ms = cloud_count_ == 0U ? -1.0 :
      std::chrono::duration<double, std::milli>(SteadyClock::now() - last_cloud_time_).count();
    const double scan_age_ms = scan_count_ == 0U ? -1.0 :
      std::chrono::duration<double, std::milli>(SteadyClock::now() - last_scan_time_).count();
    RCLCPP_INFO(
      get_logger(),
      "COLLISION_GATE front=%s approach=%s rotation=%s rear=%s "
      "counts=%d/%d/%d/%d self_filtered=%d nearest=%dmm approach_nearest=%dmm "
      "cloud_age=%.1fms scan_age=%.1fms scan_alive=%s",
      front_blocked() ? "true" : "false",
      (held(cloud_approach_until_) || held(scan_approach_until_)) ?
      "true" : "false",
      held(scan_rotation_until_) ? "true" : "false",
      held(scan_rear_until_) ? "true" : "false",
      cloud_front_count_ + scan_front_count_,
      cloud_approach_count_ + scan_approach_count_,
      scan_rotation_count_, scan_rear_count_,
      scan_self_filtered_count_,
      std::min(cloud_nearest_mm_, scan_nearest_mm_),
      std::min(cloud_approach_nearest_mm_, scan_approach_nearest_mm_),
      cloud_age_ms, scan_age_ms,
      scan_alive() ? "true" : "false");
  }

  std::string input_topic_;
  std::string scan_topic_;
  std::string stop_topic_;
  std::string status_topic_;
  double front_x_min_{0.20};
  double front_x_max_{0.62};
  double front_half_width_{0.39};
  double z_min_{0.02};
  double z_max_{1.40};
  int cloud_min_points_{6};
  double approach_x_min_{0.20};
  double approach_x_max_{1.20};
  double approach_half_width_{0.50};
  int approach_min_points_{3};
  double rear_x_min_{-0.62};
  double rear_x_max_{-0.20};
  double rear_half_width_{0.39};
  double rotation_radius_{0.52};
  double scan_self_filter_half_length_{0.33};
  double scan_self_filter_half_width_{0.33};
  int scan_min_points_{2};
  double laser_x_{0.20};
  double laser_y_{0.0};
  double laser_yaw_{0.0};
  double scan_timeout_sec_{0.35};
  double hold_sec_{0.25};
  int sample_stride_{1};
  double status_period_sec_{1.0};
  int cloud_front_count_{0};
  int scan_front_count_{0};
  int scan_rear_count_{0};
  int scan_rotation_count_{0};
  int scan_self_filtered_count_{0};
  int cloud_approach_count_{0};
  int scan_approach_count_{0};
  int cloud_nearest_mm_{9999};
  int scan_nearest_mm_{9999};
  int cloud_approach_nearest_mm_{9999};
  int scan_approach_nearest_mm_{9999};
  size_t cloud_count_{0U};
  size_t scan_count_{0U};
  SteadyClock::time_point last_cloud_time_{};
  SteadyClock::time_point last_scan_time_{};
  SteadyClock::time_point cloud_front_until_{};
  SteadyClock::time_point scan_front_until_{};
  SteadyClock::time_point scan_rear_until_{};
  SteadyClock::time_point scan_rotation_until_{};
  SteadyClock::time_point cloud_approach_until_{};
  SteadyClock::time_point scan_approach_until_{};
  rclcpp::Subscription<PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalCloudCollisionGate>());
  rclcpp::shutdown();
  return 0;
}
