#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{
using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;
using std::chrono::steady_clock;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

constexpr float kInvalidFloat = std::numeric_limits<float>::quiet_NaN();
constexpr char kPipelineVersion[] = "v6.36";
constexpr size_t kClearRaySubsample = 4U;

bool host_is_big_endian()
{
  const uint16_t value = 0x0102;
  return *(reinterpret_cast<const uint8_t *>(&value)) == 0x01;
}

uint16_t byte_swap_u16(uint16_t value)
{
  return static_cast<uint16_t>((value >> 8U) | (value << 8U));
}

float byte_swap_f32(float value)
{
  uint32_t raw = 0;
  std::memcpy(&raw, &value, sizeof(raw));
  raw = ((raw & 0x000000FFU) << 24U) |
        ((raw & 0x0000FF00U) << 8U) |
        ((raw & 0x00FF0000U) >> 8U) |
        ((raw & 0xFF000000U) >> 24U);
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}

double duration_ms(const steady_clock::time_point & start, const steady_clock::time_point & end)
{
  return std::chrono::duration<double, std::milli>(end - start).count();
}

double steady_gap_ms(
  const steady_clock::time_point & previous,
  const steady_clock::time_point & current)
{
  if (previous.time_since_epoch().count() == 0) {
    return -1.0;
  }
  return duration_ms(previous, current);
}

struct Vec3f
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
};

struct GroundPlane
{
  bool valid{false};
  double a{0.0};
  double b{0.0};
  double c{0.0};
  size_t candidates{0U};
  size_t inliers{0U};
};

struct RigidTransform
{
  std::array<float, 9> r{1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F};
  Vec3f t{};

  Vec3f apply(float x, float y, float z) const
  {
    return {
      r[0] * x + r[1] * y + r[2] * z + t.x,
      r[3] * x + r[4] * y + r[5] * z + t.y,
      r[6] * x + r[7] * y + r[8] * z + t.z,
    };
  }
};

struct RaySample
{
  uint32_t u{0U};
  uint32_t v{0U};
  float ray_x{0.0F};
  float ray_y{0.0F};
};

RigidTransform transform_from_msg(const geometry_msgs::msg::TransformStamped & transform)
{
  const auto & q = transform.transform.rotation;
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  const double x = norm > 1.0e-12 ? q.x / norm : 0.0;
  const double y = norm > 1.0e-12 ? q.y / norm : 0.0;
  const double z = norm > 1.0e-12 ? q.z / norm : 0.0;
  const double w = norm > 1.0e-12 ? q.w / norm : 1.0;

  RigidTransform out;
  out.r = {
    static_cast<float>(1.0 - 2.0 * (y * y + z * z)),
    static_cast<float>(2.0 * (x * y - z * w)),
    static_cast<float>(2.0 * (x * z + y * w)),
    static_cast<float>(2.0 * (x * y + z * w)),
    static_cast<float>(1.0 - 2.0 * (x * x + z * z)),
    static_cast<float>(2.0 * (y * z - x * w)),
    static_cast<float>(2.0 * (x * z - y * w)),
    static_cast<float>(2.0 * (y * z + x * w)),
    static_cast<float>(1.0 - 2.0 * (x * x + y * y)),
  };
  out.t = {
    static_cast<float>(transform.transform.translation.x),
    static_cast<float>(transform.transform.translation.y),
    static_cast<float>(transform.transform.translation.z),
  };
  return out;
}

PointCloud2 make_xyz_cloud(
  const std_msgs::msg::Header & header,
  const std::string & frame_id,
  const std::vector<Vec3f> & points)
{
  PointCloud2 cloud;
  cloud.header = header;
  cloud.header.frame_id = frame_id;
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(points.size());
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = 12;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(3);

  cloud.fields[0].name = "x";
  cloud.fields[0].offset = 0;
  cloud.fields[0].datatype = PointField::FLOAT32;
  cloud.fields[0].count = 1;
  cloud.fields[1].name = "y";
  cloud.fields[1].offset = 4;
  cloud.fields[1].datatype = PointField::FLOAT32;
  cloud.fields[1].count = 1;
  cloud.fields[2].name = "z";
  cloud.fields[2].offset = 8;
  cloud.fields[2].datatype = PointField::FLOAT32;
  cloud.fields[2].count = 1;

  cloud.data.resize(points.size() * cloud.point_step);
  if (!points.empty()) {
    static_assert(sizeof(Vec3f) == 12, "Vec3f must be tightly packed");
    std::memcpy(cloud.data.data(), points.data(), points.size() * sizeof(Vec3f));
  }
  return cloud;
}

Marker cube_marker(
  int id,
  const std::string & ns,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const std::array<double, 3> & minimum,
  const std::array<double, 3> & maximum,
  const std::array<float, 4> & color)
{
  Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = Marker::CUBE;
  marker.action = Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.pose.position.x = (minimum[0] + maximum[0]) * 0.5;
  marker.pose.position.y = (minimum[1] + maximum[1]) * 0.5;
  marker.pose.position.z = (minimum[2] + maximum[2]) * 0.5;
  marker.scale.x = maximum[0] - minimum[0];
  marker.scale.y = maximum[1] - minimum[1];
  marker.scale.z = maximum[2] - minimum[2];
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];
  const auto lifetime = rclcpp::Duration::from_seconds(1.5);
  marker.lifetime.sec = static_cast<int32_t>(lifetime.nanoseconds() / 1000000000LL);
  marker.lifetime.nanosec = static_cast<uint32_t>(lifetime.nanoseconds() % 1000000000LL);
  return marker;
}

class MetricWindow
{
public:
  explicit MetricWindow(size_t capacity = 300U)
  : values_(std::max<size_t>(capacity, 1U), 0.0)
  {
  }

  void add(double value)
  {
    if (!std::isfinite(value) || value < 0.0) {
      return;
    }
    values_[next_] = value;
    next_ = (next_ + 1U) % values_.size();
    count_ = std::min(count_ + 1U, values_.size());
  }

  struct Snapshot
  {
    double average{0.0};
    double p95{0.0};
    double maximum{0.0};
    size_t count{0U};
  };

  Snapshot snapshot() const
  {
    Snapshot out;
    out.count = count_;
    if (count_ == 0U) {
      return out;
    }

    std::vector<double> copy;
    copy.reserve(count_);
    double sum = 0.0;
    for (size_t i = 0U; i < count_; ++i) {
      const double value = values_[i];
      copy.push_back(value);
      sum += value;
      out.maximum = std::max(out.maximum, value);
    }
    out.average = sum / static_cast<double>(count_);
    std::sort(copy.begin(), copy.end());
    const size_t index = static_cast<size_t>(
      std::ceil(0.95 * static_cast<double>(copy.size()))) - 1U;
    out.p95 = copy[std::min(index, copy.size() - 1U)];
    return out;
  }

private:
  std::vector<double> values_;
  size_t next_{0U};
  size_t count_{0U};
};
}  // namespace

class DepthImageToLocalCloudNode final : public rclcpp::Node
{
public:
  DepthImageToLocalCloudNode()
  : Node("depth_image_to_local_cloud"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    pipeline_version_ = declare_parameter<std::string>(
      "pipeline_version", kPipelineVersion);
    if (pipeline_version_ != kPipelineVersion) {
      throw std::runtime_error(
              "pipeline_version is compile-time owned; expected " +
              std::string(kPipelineVersion) + ", got " + pipeline_version_);
    }
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
    camera_info_topic_ = declare_parameter<std::string>(
      "camera_info_topic", "/camera/depth/camera_info");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/local_highres_cloud_v21");
    sensor_output_topic_ = declare_parameter<std::string>(
      "sensor_output_topic", "/local_highres_cloud_v21/sensor");
    persistent_sensor_output_topic_ = declare_parameter<std::string>(
      "persistent_sensor_output_topic", "/local_highres_cloud_v21/persistent_sensor");
    immediate_obstacle_output_topic_ = declare_parameter<std::string>(
      "immediate_obstacle_output_topic", "/local_highres_cloud_v21/immediate_obstacles");
    clear_sensor_output_topic_ = declare_parameter<std::string>(
      "clear_sensor_output_topic", "/local_highres_cloud_v21/clear_sensor");
    stats_topic_ = declare_parameter<std::string>(
      "stats_topic", "/local_highres_cloud_v21/stats");
    marker_topic_ = declare_parameter<std::string>(
      "marker_topic", "/local_highres_cloud_v21/crop_markers");
    output_frame_ = declare_parameter<std::string>("output_frame", "base_link");

    max_rate_hz_ = declare_parameter<double>("max_rate_hz", 30.0);
    pixel_stride_ = std::max(1, static_cast<int>(declare_parameter<int>("pixel_stride", 2)));
    depth_unit_scale_ = declare_parameter<double>("depth_unit_scale", 0.001);
    min_range_ = declare_parameter<double>("min_range", 0.20);
    max_range_ = declare_parameter<double>("max_range", 4.0);
    voxel_size_ = declare_parameter<double>("voxel_size", 0.03);
    spatial_filter_enabled_ = declare_parameter<bool>("spatial_filter_enabled", true);
    spatial_depth_threshold_m_ = declare_parameter<double>(
      "spatial_depth_threshold_m", 0.08);
    spatial_depth_threshold_ratio_ = declare_parameter<double>(
      "spatial_depth_threshold_ratio", 0.025);
    spatial_min_neighbors_ = std::max(
      0, static_cast<int>(declare_parameter<int>("spatial_min_neighbors", 2)));
    temporal_filter_enabled_ = declare_parameter<bool>("temporal_filter_enabled", true);
    temporal_alpha_ = declare_parameter<double>("temporal_alpha", 0.65);
    temporal_max_delta_m_ = declare_parameter<double>("temporal_max_delta_m", 0.06);
    voxel_outlier_filter_enabled_ = declare_parameter<bool>(
      "voxel_outlier_filter_enabled", true);
    voxel_min_neighbors_ = std::max(
      0, static_cast<int>(declare_parameter<int>("voxel_min_neighbors", 1)));
    persistent_mark_confirmation_enabled_ = declare_parameter<bool>(
      "persistent_mark_confirmation_enabled", true);
    persistent_mark_confirmation_frames_ = std::clamp(
      static_cast<int>(declare_parameter<int64_t>(
        "persistent_mark_confirmation_frames", 3)), 1, 255);
    persistent_mark_max_gap_frames_ = std::clamp(
      static_cast<int>(declare_parameter<int64_t>(
        "persistent_mark_max_gap_frames", 1)), 1, 30);
    persistent_mark_neighbor_radius_ = std::clamp(
      static_cast<int>(declare_parameter<int64_t>(
        "persistent_mark_neighbor_radius", 1)), 0, 2);
    persistent_geometry_guard_enabled_ = declare_parameter<bool>(
      "persistent_geometry_guard_enabled", true);
    recent_mark_ground_guard_height_m_ = declare_parameter<double>(
      "recent_mark_ground_guard_height_m", 0.12);
    recent_mark_min_vertical_span_m_ = declare_parameter<double>(
      "recent_mark_min_vertical_span_m", 0.025);
    persistent_mark_ground_guard_height_m_ = declare_parameter<double>(
      "persistent_mark_ground_guard_height_m", 0.15);
    persistent_mark_min_vertical_span_m_ = declare_parameter<double>(
      "persistent_mark_min_vertical_span_m", 0.04);
    mark_geometry_neighbor_radius_ = std::clamp(
      static_cast<int>(declare_parameter<int64_t>(
        "mark_geometry_neighbor_radius", 1)), 0, 3);
    transform_timeout_sec_ = declare_parameter<double>("transform_timeout", 0.50);
    max_input_age_ms_ = declare_parameter<double>("max_input_age_ms", 150.0);
    min_clear_valid_depth_ratio_ = declare_parameter<double>(
      "min_clear_valid_depth_ratio", 0.05);

    roi_u_min_ = declare_parameter<int>("roi_u_min", 0);
    roi_u_max_ = declare_parameter<int>("roi_u_max", -1);
    roi_v_min_ = declare_parameter<int>("roi_v_min", 0);
    roi_v_max_ = declare_parameter<int>("roi_v_max", -1);

    x_min_ = declare_parameter<double>("x_min", 0.15);
    x_max_ = declare_parameter<double>("x_max", 4.00);
    y_min_ = declare_parameter<double>("y_min", -2.50);
    y_max_ = declare_parameter<double>("y_max", 2.50);
    z_min_ = declare_parameter<double>("z_min", -0.50);
    z_max_ = declare_parameter<double>("z_max", 2.00);

    remove_self_ = declare_parameter<bool>("remove_self", true);
    self_x_min_ = declare_parameter<double>("self_x_min", -0.36);
    self_x_max_ = declare_parameter<double>("self_x_max", 0.36);
    self_y_min_ = declare_parameter<double>("self_y_min", -0.36);
    self_y_max_ = declare_parameter<double>("self_y_max", 0.36);
    self_z_min_ = declare_parameter<double>("self_z_min", -0.10);
    self_z_max_ = declare_parameter<double>("self_z_max", 0.90);

    ground_filter_enabled_ = declare_parameter<bool>("ground_filter_enabled", false);
    ground_z_min_ = declare_parameter<double>("ground_z_min", -0.06);
    ground_z_max_ = declare_parameter<double>("ground_z_max", 0.08);
    adaptive_ground_plane_ = declare_parameter<bool>("adaptive_ground_plane", true);
    ground_plane_candidate_min_z_ = declare_parameter<double>(
      "ground_plane_candidate_min_z", -0.15);
    ground_plane_candidate_max_z_ = declare_parameter<double>(
      "ground_plane_candidate_max_z", 0.12);
    ground_plane_fit_tolerance_ = declare_parameter<double>(
      "ground_plane_fit_tolerance", 0.025);
    ground_plane_seed_tolerance_ = declare_parameter<double>(
      "ground_plane_seed_tolerance", 0.06);
    ground_plane_temporal_alpha_ = declare_parameter<double>(
      "ground_plane_temporal_alpha", 0.18);
    ground_plane_max_slope_step_ = declare_parameter<double>(
      "ground_plane_max_slope_step", 0.02);
    ground_plane_max_offset_step_ = declare_parameter<double>(
      "ground_plane_max_offset_step", 0.02);
    ground_plane_remove_below_ = declare_parameter<double>(
      "ground_plane_remove_below", 0.035);
    ground_plane_remove_above_ = declare_parameter<double>(
      "ground_plane_remove_above", 0.025);
    ground_plane_max_slope_ = declare_parameter<double>(
      "ground_plane_max_slope", 0.12);
    ground_plane_min_inliers_ = std::max(
      50, static_cast<int>(declare_parameter<int64_t>("ground_plane_min_inliers", 120)));
    ground_plane_min_inlier_ratio_ = declare_parameter<double>(
      "ground_plane_min_inlier_ratio", 0.30);
    ground_speckle_max_height_ = declare_parameter<double>(
      "ground_speckle_max_height", 0.06);
    ground_speckle_min_neighbors_ = std::max(
      1, static_cast<int>(declare_parameter<int64_t>("ground_speckle_min_neighbors", 4)));
    publish_markers_ = declare_parameter<bool>("publish_markers", true);
    stats_period_sec_ = declare_parameter<double>("stats_period_sec", 1.0);
    stats_window_size_ = std::max(30, static_cast<int>(declare_parameter<int>("stats_window_size", 300)));
    process_warn_ms_ = declare_parameter<double>("process_warn_ms", 50.0);
    age_warn_ms_ = declare_parameter<double>("age_warn_ms", 120.0);
    stall_warn_gap_ms_ = declare_parameter<double>("stall_warn_gap_ms", 120.0);

    validate_parameters();
    initialize_voxel_table();
    initialize_metric_windows();

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    const auto marker_qos = rclcpp::QoS(1).reliable().transient_local();

    cloud_pub_ = create_publisher<PointCloud2>(output_topic_, sensor_qos);
    sensor_cloud_pub_ = create_publisher<PointCloud2>(sensor_output_topic_, sensor_qos);
    persistent_sensor_cloud_pub_ = create_publisher<PointCloud2>(
      persistent_sensor_output_topic_, sensor_qos);
    immediate_obstacle_cloud_pub_ = create_publisher<PointCloud2>(
      immediate_obstacle_output_topic_, sensor_qos);
    clear_sensor_cloud_pub_ = create_publisher<PointCloud2>(
      clear_sensor_output_topic_, sensor_qos);
    stats_pub_ = create_publisher<std_msgs::msg::String>(stats_topic_, rclcpp::QoS(10));
    marker_pub_ = create_publisher<MarkerArray>(marker_topic_, marker_qos);

    camera_info_sub_ = create_subscription<CameraInfo>(
      camera_info_topic_, sensor_qos,
      std::bind(&DepthImageToLocalCloudNode::camera_info_callback, this, std::placeholders::_1));
    depth_sub_ = create_subscription<Image>(
      depth_topic_, sensor_qos,
      std::bind(&DepthImageToLocalCloudNode::depth_callback, this, std::placeholders::_1));

    marker_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&DepthImageToLocalCloudNode::publish_markers, this));
    stats_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(std::max(0.2, stats_period_sec_))),
      std::bind(&DepthImageToLocalCloudNode::publish_stats, this));

    worker_thread_ = std::thread(&DepthImageToLocalCloudNode::worker_loop, this);

    RCLCPP_INFO(
      get_logger(),
      "STEP10V2.1 stable depth-image cloud started: %s + %s -> %s, frame=%s, "
      "stride=%d, voxel=%.3fm, max_rate=%.1fHz, spatial=%s temporal=%s "
      "voxel_outlier=%s, temporal_mark=%s/%d frames, "
      "recent_guard=%.3fm/%.3fm, persistent_guard=%.3fm/%.3fm, "
      "clear_valid_ratio>=%.3f, pipeline=%s",
      depth_topic_.c_str(), camera_info_topic_.c_str(), output_topic_.c_str(),
      output_frame_.c_str(), pixel_stride_, voxel_size_, max_rate_hz_,
      spatial_filter_enabled_ ? "on" : "off",
      temporal_filter_enabled_ ? "on" : "off",
      voxel_outlier_filter_enabled_ ? "on" : "off",
      persistent_mark_confirmation_enabled_ ? "confirmed" : "immediate",
      persistent_mark_confirmation_frames_,
      recent_mark_ground_guard_height_m_, recent_mark_min_vertical_span_m_,
      persistent_mark_ground_guard_height_m_, persistent_mark_min_vertical_span_m_,
      min_clear_valid_depth_ratio_, pipeline_version_.c_str());
  }

  ~DepthImageToLocalCloudNode() override
  {
    stop_worker_.store(true);
    mailbox_cv_.notify_all();
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
  }

private:
  struct PendingFrame
  {
    Image::ConstSharedPtr image;
    steady_clock::time_point arrival_steady{};
    double arrival_age_ms{-1.0};
  };

  void validate_parameters() const
  {
    if (!(min_range_ >= 0.0) || (max_range_ > 0.0 && min_range_ >= max_range_)) {
      throw std::invalid_argument("min_range/max_range invalid");
    }
    if (!(x_min_ < x_max_ && y_min_ < y_max_ && z_min_ < z_max_)) {
      throw std::invalid_argument("local crop min must be smaller than max");
    }
    if (!(self_x_min_ < self_x_max_ && self_y_min_ < self_y_max_ &&
      self_z_min_ < self_z_max_))
    {
      throw std::invalid_argument("self-filter min must be smaller than max");
    }
    if (!(ground_z_min_ < ground_z_max_)) {
      throw std::invalid_argument("ground_z_min must be smaller than ground_z_max");
    }
    if (!(ground_plane_candidate_min_z_ < ground_plane_candidate_max_z_) ||
      ground_plane_fit_tolerance_ <= 0.0 || ground_plane_remove_below_ < 0.0 ||
      ground_plane_remove_above_ < 0.0 || ground_plane_max_slope_ <= 0.0 ||
      ground_plane_seed_tolerance_ <= ground_plane_fit_tolerance_ ||
      ground_plane_temporal_alpha_ <= 0.0 || ground_plane_temporal_alpha_ > 1.0 ||
      ground_plane_max_slope_step_ <= 0.0 || ground_plane_max_offset_step_ <= 0.0 ||
      ground_plane_min_inlier_ratio_ <= 0.0 || ground_plane_min_inlier_ratio_ > 1.0 ||
      ground_speckle_max_height_ <= ground_plane_remove_above_ ||
      recent_mark_ground_guard_height_m_ <= ground_plane_remove_above_ ||
      persistent_mark_ground_guard_height_m_ < recent_mark_ground_guard_height_m_ ||
      recent_mark_min_vertical_span_m_ <= 0.0 ||
      persistent_mark_min_vertical_span_m_ < recent_mark_min_vertical_span_m_)
    {
      throw std::invalid_argument("adaptive ground-plane parameters invalid");
    }
    if (!(depth_unit_scale_ > 0.0)) {
      throw std::invalid_argument("depth_unit_scale must be positive");
    }
    if (voxel_size_ < 0.0) {
      throw std::invalid_argument("voxel_size cannot be negative");
    }
    if (spatial_depth_threshold_m_ < 0.0 || spatial_depth_threshold_ratio_ < 0.0) {
      throw std::invalid_argument("spatial depth thresholds cannot be negative");
    }
    if (!(temporal_alpha_ >= 0.0 && temporal_alpha_ <= 1.0) ||
      temporal_max_delta_m_ < 0.0)
    {
      throw std::invalid_argument("temporal filter parameters invalid");
    }
    if (!(min_clear_valid_depth_ratio_ >= 0.0 && min_clear_valid_depth_ratio_ <= 1.0)) {
      throw std::invalid_argument("min_clear_valid_depth_ratio must be within [0, 1]");
    }
  }

  void initialize_voxel_table()
  {
    if (!(voxel_size_ > 0.0)) {
      return;
    }
    voxel_nx_ = std::max<int64_t>(
      1, static_cast<int64_t>(std::ceil((x_max_ - x_min_) / voxel_size_)));
    voxel_ny_ = std::max<int64_t>(
      1, static_cast<int64_t>(std::ceil((y_max_ - y_min_) / voxel_size_)));
    voxel_nz_ = std::max<int64_t>(
      1, static_cast<int64_t>(std::ceil((z_max_ - z_min_) / voxel_size_)));

    const uint64_t total = static_cast<uint64_t>(voxel_nx_) *
      static_cast<uint64_t>(voxel_ny_) * static_cast<uint64_t>(voxel_nz_);
    if (total > 100000000ULL) {
      throw std::invalid_argument("voxel table is too large; increase voxel_size or shrink crop");
    }
    voxel_generation_.assign(static_cast<size_t>(total), 0U);
    persistent_mark_last_seen_generation_.assign(static_cast<size_t>(total), 0U);
    persistent_mark_hit_count_.assign(static_cast<size_t>(total), 0U);
    const uint64_t columns = static_cast<uint64_t>(voxel_nx_) *
      static_cast<uint64_t>(voxel_ny_);
    column_generation_.assign(static_cast<size_t>(columns), 0U);
    column_min_residual_.assign(static_cast<size_t>(columns), 0.0F);
    column_max_residual_.assign(static_cast<size_t>(columns), 0.0F);
    const uint64_t table_bytes = total *
      (2U * sizeof(uint32_t) + sizeof(uint8_t)) +
      columns * (sizeof(uint32_t) + 2U * sizeof(float));
    RCLCPP_INFO(
      get_logger(),
      "Preallocated temporal voxel tables: %ld x %ld x %ld = %llu cells (%.1f MiB)",
      static_cast<long>(voxel_nx_), static_cast<long>(voxel_ny_),
      static_cast<long>(voxel_nz_), static_cast<unsigned long long>(total),
      static_cast<double>(table_bytes) / (1024.0 * 1024.0));
  }

  void initialize_metric_windows()
  {
    process_window_ = std::make_unique<MetricWindow>(static_cast<size_t>(stats_window_size_));
    age_window_ = std::make_unique<MetricWindow>(static_cast<size_t>(stats_window_size_));
    input_gap_window_ = std::make_unique<MetricWindow>(static_cast<size_t>(stats_window_size_));
    arrival_gap_window_ = std::make_unique<MetricWindow>(static_cast<size_t>(stats_window_size_));
    output_gap_window_ = std::make_unique<MetricWindow>(static_cast<size_t>(stats_window_size_));
  }

  void camera_info_callback(const CameraInfo::SharedPtr msg)
  {
    if (!(msg->k[0] > 0.0 && msg->k[4] > 0.0)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring invalid CameraInfo with fx=%.3f fy=%.3f", msg->k[0], msg->k[4]);
      return;
    }
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    bool changed = !latest_camera_info_.has_value();
    if (!changed) {
      changed = latest_camera_info_->width != msg->width ||
        latest_camera_info_->height != msg->height;
      for (size_t i = 0U; i < msg->k.size() && !changed; ++i) {
        changed = std::abs(latest_camera_info_->k[i] - msg->k[i]) > 1.0e-9;
      }
    }
    latest_camera_info_ = *msg;
    if (changed) {
      ++camera_info_version_;
    }
  }

  std::optional<std::pair<CameraInfo, uint64_t>> latest_camera_info() const
  {
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    if (!latest_camera_info_.has_value()) {
      return std::nullopt;
    }
    return std::make_pair(*latest_camera_info_, camera_info_version_);
  }

  double image_age_ms(const builtin_interfaces::msg::Time & stamp) const
  {
    if (stamp.sec == 0 && stamp.nanosec == 0) {
      return -1.0;
    }
    const rclcpp::Time image_time(stamp, get_clock()->get_clock_type());
    return (now() - image_time).seconds() * 1000.0;
  }

  int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp) const
  {
    return static_cast<int64_t>(stamp.sec) * 1000000000LL +
      static_cast<int64_t>(stamp.nanosec);
  }

  void depth_callback(const Image::ConstSharedPtr image)
  {
    const auto arrival = steady_clock::now();
    const double arrival_age = image_age_ms(image->header.stamp);
    const int64_t current_stamp_ns = stamp_ns(image->header.stamp);

    {
      std::lock_guard<std::mutex> stats_lock(stats_mutex_);
      ++received_frames_;
      const double arrival_gap = steady_gap_ms(last_input_arrival_steady_, arrival);
      if (arrival_gap >= 0.0) {
        arrival_gap_window_->add(arrival_gap);
        latest_arrival_gap_ms_ = arrival_gap;
      }
      last_input_arrival_steady_ = arrival;

      if (last_input_stamp_ns_ != 0 && current_stamp_ns != 0) {
        const double message_gap = static_cast<double>(current_stamp_ns - last_input_stamp_ns_) / 1.0e6;
        if (current_stamp_ns == last_input_stamp_ns_) {
          ++duplicate_timestamp_frames_;
        } else if (current_stamp_ns < last_input_stamp_ns_) {
          ++backward_timestamp_frames_;
        } else {
          input_gap_window_->add(message_gap);
          latest_input_gap_ms_ = message_gap;
          if (message_gap > stall_warn_gap_ms_) {
            ++input_stall_events_;
          }
        }
      }
      if (current_stamp_ns != 0) {
        last_input_stamp_ns_ = current_stamp_ns;
      }
      latest_input_age_ms_ = arrival_age;
      latest_image_width_ = image->width;
      latest_image_height_ = image->height;
      latest_encoding_ = image->encoding;
      latest_source_frame_ = image->header.frame_id;
    }

    {
      std::lock_guard<std::mutex> lock(mailbox_mutex_);
      if (latest_pending_.has_value()) {
        ++mailbox_replaced_frames_;
      }
      latest_pending_ = PendingFrame{image, arrival, arrival_age};
    }
    mailbox_cv_.notify_one();
  }

  bool ensure_cached_transform(const std::string & source_frame, double & lookup_ms)
  {
    lookup_ms = 0.0;
    if (source_frame.empty() || source_frame == output_frame_) {
      cached_transform_ = RigidTransform{};
      cached_transform_source_frame_ = source_frame;
      transform_cached_ = true;
      return true;
    }
    if (transform_cached_ && cached_transform_source_frame_ == source_frame) {
      return true;
    }

    const auto start = steady_clock::now();
    try {
      const auto transform = tf_buffer_.lookupTransform(
        output_frame_, source_frame,
        rclcpp::Time(0, 0, get_clock()->get_clock_type()),
        rclcpp::Duration::from_seconds(transform_timeout_sec_));
      cached_transform_ = transform_from_msg(transform);
      cached_transform_source_frame_ = source_frame;
      transform_cached_ = true;
      lookup_ms = duration_ms(start, steady_clock::now());
      ++tf_cache_refreshes_;
      RCLCPP_INFO(
        get_logger(), "Cached static TF once: %s <- %s (%.2f ms)",
        output_frame_.c_str(), source_frame.c_str(), lookup_ms);
      return true;
    } catch (const tf2::TransformException & error) {
      lookup_ms = duration_ms(start, steady_clock::now());
      ++tf_dropped_frames_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting to cache TF %s <- %s: %s", output_frame_.c_str(),
        source_frame.c_str(), error.what());
      return false;
    }
  }

  bool rebuild_ray_table_if_needed(
    const Image & image,
    const CameraInfo & camera_info,
    uint64_t camera_info_version)
  {
    const bool same = ray_table_ready_ &&
      ray_image_width_ == image.width && ray_image_height_ == image.height &&
      ray_camera_info_version_ == camera_info_version;
    if (same) {
      return true;
    }

    const double sx = camera_info.width > 0U ?
      static_cast<double>(image.width) / static_cast<double>(camera_info.width) : 1.0;
    const double sy = camera_info.height > 0U ?
      static_cast<double>(image.height) / static_cast<double>(camera_info.height) : 1.0;
    const double fx = camera_info.k[0] * sx;
    const double fy = camera_info.k[4] * sy;
    const double cx = camera_info.k[2] * sx;
    const double cy = camera_info.k[5] * sy;
    if (!(fx > 0.0 && fy > 0.0)) {
      return false;
    }

    const int u_begin = std::clamp(roi_u_min_, 0, static_cast<int>(image.width) - 1);
    const int u_end = roi_u_max_ < 0 ? static_cast<int>(image.width) :
      std::clamp(roi_u_max_, u_begin + 1, static_cast<int>(image.width));
    const int v_begin = std::clamp(roi_v_min_, 0, static_cast<int>(image.height) - 1);
    const int v_end = roi_v_max_ < 0 ? static_cast<int>(image.height) :
      std::clamp(roi_v_max_, v_begin + 1, static_cast<int>(image.height));

    const size_t capacity =
      static_cast<size_t>((u_end - u_begin + pixel_stride_ - 1) / pixel_stride_) *
      static_cast<size_t>((v_end - v_begin + pixel_stride_ - 1) / pixel_stride_);
    ray_table_.clear();
    ray_table_.reserve(capacity);
    for (int v = v_begin; v < v_end; v += pixel_stride_) {
      const float ray_y = static_cast<float>((static_cast<double>(v) - cy) / fy);
      for (int u = u_begin; u < u_end; u += pixel_stride_) {
        ray_table_.push_back(RaySample{
          static_cast<uint32_t>(u), static_cast<uint32_t>(v),
          static_cast<float>((static_cast<double>(u) - cx) / fx), ray_y});
      }
    }

    points_buffer_.clear();
    // 预留完整采样容量，避免某些纹理丰富帧突然扩容造成卡顿尖峰。
    points_buffer_.reserve(ray_table_.size());
    sensor_points_buffer_.clear();
    sensor_points_buffer_.reserve(ray_table_.size());
    immediate_obstacle_points_buffer_.clear();
    immediate_obstacle_points_buffer_.reserve(ray_table_.size());
    confirmed_sensor_points_buffer_.clear();
    confirmed_sensor_points_buffer_.reserve(ray_table_.size());
    persistent_sensor_points_buffer_.clear();
    persistent_sensor_points_buffer_.reserve(ray_table_.size());
    raw_points_buffer_.clear();
    raw_points_buffer_.reserve(ray_table_.size());
    raw_sensor_points_buffer_.clear();
    raw_sensor_points_buffer_.reserve(ray_table_.size());
    clear_sensor_points_buffer_.clear();
    clear_sensor_points_buffer_.reserve(
      (ray_table_.size() + kClearRaySubsample - 1U) / kClearRaySubsample);
    voxel_keys_buffer_.clear();
    voxel_keys_buffer_.reserve(ray_table_.size());
    temporal_depth_buffer_.assign(ray_table_.size(), kInvalidFloat);
    ray_image_width_ = image.width;
    ray_image_height_ = image.height;
    ray_camera_info_version_ = camera_info_version;
    ray_intrinsics_scaled_ = camera_info.width != image.width || camera_info.height != image.height;
    ray_table_ready_ = true;
    ++ray_table_rebuilds_;

    RCLCPP_INFO(
      get_logger(),
      "Precomputed %zu projection rays for %ux%u stride=%d%s",
      ray_table_.size(), image.width, image.height, pixel_stride_,
      ray_intrinsics_scaled_.load() ? " [scaled CameraInfo]" : "");
    return true;
  }

  float read_depth_m(const Image & image, const uint8_t * ptr) const
  {
    const bool swap = image.is_bigendian != host_is_big_endian();
    if (image.encoding == "16UC1" || image.encoding == "mono16") {
      uint16_t raw = 0;
      std::memcpy(&raw, ptr, sizeof(raw));
      if (swap) {
        raw = byte_swap_u16(raw);
      }
      if (raw == 0U) {
        return kInvalidFloat;
      }
      return static_cast<float>(static_cast<double>(raw) * depth_unit_scale_);
    }
    if (image.encoding == "32FC1") {
      float raw = 0.0F;
      std::memcpy(&raw, ptr, sizeof(raw));
      if (swap) {
        raw = byte_swap_f32(raw);
      }
      return raw;
    }
    return kInvalidFloat;
  }

  int bytes_per_pixel(const std::string & encoding) const
  {
    if (encoding == "16UC1" || encoding == "mono16") {
      return 2;
    }
    if (encoding == "32FC1") {
      return 4;
    }
    return 0;
  }

  bool inside_self(const Vec3f & p) const
  {
    return p.x >= self_x_min_ && p.x <= self_x_max_ &&
           p.y >= self_y_min_ && p.y <= self_y_max_ &&
           p.z >= self_z_min_ && p.z <= self_z_max_;
  }

  bool inside_crop(const Vec3f & p) const
  {
    return p.x >= x_min_ && p.x <= x_max_ &&
           p.y >= y_min_ && p.y <= y_max_ &&
           p.z >= z_min_ && p.z <= z_max_;
  }

  std::optional<size_t> mark_voxel_first_hit(const Vec3f & point)
  {
    if (!(voxel_size_ > 0.0)) {
      return std::numeric_limits<size_t>::max();
    }
    const int64_t ix = static_cast<int64_t>(std::floor((point.x - x_min_) / voxel_size_));
    const int64_t iy = static_cast<int64_t>(std::floor((point.y - y_min_) / voxel_size_));
    const int64_t iz = static_cast<int64_t>(std::floor((point.z - z_min_) / voxel_size_));
    if (ix < 0 || iy < 0 || iz < 0 ||
      ix >= voxel_nx_ || iy >= voxel_ny_ || iz >= voxel_nz_)
    {
      return std::nullopt;
    }
    const size_t key = static_cast<size_t>(ix + voxel_nx_ * (iy + voxel_ny_ * iz));
    if (voxel_generation_[key] == current_voxel_generation_) {
      return std::nullopt;
    }
    voxel_generation_[key] = current_voxel_generation_;
    return key;
  }

  int occupied_voxel_neighbors(size_t key) const
  {
    if (!(voxel_size_ > 0.0) || key == std::numeric_limits<size_t>::max()) {
      return voxel_min_neighbors_;
    }
    const int64_t plane = voxel_nx_ * voxel_ny_;
    const int64_t iz = static_cast<int64_t>(key) / plane;
    const int64_t remainder = static_cast<int64_t>(key) - iz * plane;
    const int64_t iy = remainder / voxel_nx_;
    const int64_t ix = remainder - iy * voxel_nx_;
    int neighbors = 0;
    for (int dz = -1; dz <= 1; ++dz) {
      const int64_t nz = iz + dz;
      if (nz < 0 || nz >= voxel_nz_) {
        continue;
      }
      for (int dy = -1; dy <= 1; ++dy) {
        const int64_t ny = iy + dy;
        if (ny < 0 || ny >= voxel_ny_) {
          continue;
        }
        for (int dx = -1; dx <= 1; ++dx) {
          const int64_t nx = ix + dx;
          if ((dx == 0 && dy == 0 && dz == 0) || nx < 0 || nx >= voxel_nx_) {
            continue;
          }
          const size_t neighbor_key = static_cast<size_t>(
            nx + voxel_nx_ * (ny + voxel_ny_ * nz));
          if (voxel_generation_[neighbor_key] == current_voxel_generation_) {
            ++neighbors;
          }
        }
      }
    }
    return neighbors;
  }

  uint8_t persistent_mark_confirmation_count(size_t key) const
  {
    if (!persistent_mark_confirmation_enabled_ ||
      persistent_mark_confirmation_frames_ <= 1 ||
      !(voxel_size_ > 0.0) ||
      key == std::numeric_limits<size_t>::max())
    {
      return static_cast<uint8_t>(
        std::min(persistent_mark_confirmation_frames_, 255));
    }

    const int64_t plane = voxel_nx_ * voxel_ny_;
    const int64_t iz = static_cast<int64_t>(key) / plane;
    const int64_t remainder = static_cast<int64_t>(key) - iz * plane;
    const int64_t iy = remainder / voxel_nx_;
    const int64_t ix = remainder - iy * voxel_nx_;
    uint8_t best_previous = 0U;

    for (int dz = -persistent_mark_neighbor_radius_;
      dz <= persistent_mark_neighbor_radius_; ++dz)
    {
      const int64_t nz = iz + dz;
      if (nz < 0 || nz >= voxel_nz_) {
        continue;
      }
      for (int dy = -persistent_mark_neighbor_radius_;
        dy <= persistent_mark_neighbor_radius_; ++dy)
      {
        const int64_t ny = iy + dy;
        if (ny < 0 || ny >= voxel_ny_) {
          continue;
        }
        for (int dx = -persistent_mark_neighbor_radius_;
          dx <= persistent_mark_neighbor_radius_; ++dx)
        {
          const int64_t nx = ix + dx;
          if (nx < 0 || nx >= voxel_nx_) {
            continue;
          }
          const size_t neighbor_key = static_cast<size_t>(
            nx + voxel_nx_ * (ny + voxel_ny_ * nz));
          const uint32_t previous_generation =
            persistent_mark_last_seen_generation_[neighbor_key];
          if (previous_generation == 0U ||
            previous_generation >= current_voxel_generation_)
          {
            continue;
          }
          const uint32_t gap = current_voxel_generation_ - previous_generation;
          if (gap <= static_cast<uint32_t>(persistent_mark_max_gap_frames_)) {
            best_previous = std::max(
              best_previous, persistent_mark_hit_count_[neighbor_key]);
          }
        }
      }
    }

    return static_cast<uint8_t>(
      std::min<int>(static_cast<int>(best_previous) + 1, 255));
  }

  void build_current_vertical_columns(const GroundPlane & ground_plane)
  {
    if (!persistent_geometry_guard_enabled_ || !(voxel_size_ > 0.0) ||
      column_generation_.empty())
    {
      return;
    }

    const size_t column_count = static_cast<size_t>(voxel_nx_ * voxel_ny_);
    for (size_t index = 0U; index < voxel_keys_buffer_.size(); ++index) {
      const size_t key = voxel_keys_buffer_[index];
      if (key == std::numeric_limits<size_t>::max()) {
        continue;
      }
      const size_t column = key % column_count;
      const Vec3f & point = points_buffer_[index];
      const float residual = static_cast<float>(
        ground_plane.valid ?
        static_cast<double>(point.z) -
        (ground_plane.a * point.x + ground_plane.b * point.y + ground_plane.c) :
        static_cast<double>(point.z));
      if (column_generation_[column] != current_voxel_generation_) {
        column_generation_[column] = current_voxel_generation_;
        column_min_residual_[column] = residual;
        column_max_residual_[column] = residual;
      } else {
        column_min_residual_[column] = std::min(column_min_residual_[column], residual);
        column_max_residual_[column] = std::max(column_max_residual_[column], residual);
      }
    }
  }

  double nearby_vertical_span(size_t key) const
  {
    if (!(voxel_size_ > 0.0) || column_generation_.empty() ||
      key == std::numeric_limits<size_t>::max())
    {
      return 0.0;
    }

    const int64_t plane = voxel_nx_ * voxel_ny_;
    const int64_t column = static_cast<int64_t>(key % static_cast<size_t>(plane));
    const int64_t iy = column / voxel_nx_;
    const int64_t ix = column - iy * voxel_nx_;
    double span = 0.0;
    for (int dy = -mark_geometry_neighbor_radius_;
      dy <= mark_geometry_neighbor_radius_; ++dy)
    {
      const int64_t ny = iy + dy;
      if (ny < 0 || ny >= voxel_ny_) {
        continue;
      }
      for (int dx = -mark_geometry_neighbor_radius_;
        dx <= mark_geometry_neighbor_radius_; ++dx)
      {
        const int64_t nx = ix + dx;
        if (nx < 0 || nx >= voxel_nx_) {
          continue;
        }
        const size_t neighbor_column = static_cast<size_t>(nx + voxel_nx_ * ny);
        if (column_generation_[neighbor_column] != current_voxel_generation_) {
          continue;
        }
        span = std::max(
          span,
          static_cast<double>(
            column_max_residual_[neighbor_column] - column_min_residual_[neighbor_column]));
      }
    }
    return span;
  }

  bool geometry_qualified_mark(
    size_t index, const GroundPlane & ground_plane,
    double clear_height, double minimum_vertical_span) const
  {
    if (!persistent_geometry_guard_enabled_) {
      return true;
    }
    const Vec3f & point = points_buffer_[index];
    const double residual = ground_plane.valid ?
      static_cast<double>(point.z) -
      (ground_plane.a * point.x + ground_plane.b * point.y + ground_plane.c) :
      static_cast<double>(point.z);

    // Never promote a point still inside the fitted floor band. A clearly high
    // return is accepted directly; lower returns must have a vertical edge in
    // the current neighborhood. Broad, repeatable tile-depth ripples therefore
    // cannot become navigation memory merely by surviving several frames.
    if (residual <= ground_plane_remove_above_) {
      return false;
    }
    if (residual >= clear_height) {
      return true;
    }
    return nearby_vertical_span(voxel_keys_buffer_[index]) >= minimum_vertical_span;
  }

  void build_persistent_mark_cloud(const GroundPlane & ground_plane)
  {
    immediate_obstacle_points_buffer_.clear();
    confirmed_sensor_points_buffer_.clear();
    persistent_sensor_points_buffer_.clear();
    latest_temporally_confirmed_mark_points_.store(0U);
    persistent_mark_counts_buffer_.resize(points_buffer_.size());
    build_current_vertical_columns(ground_plane);

    // The hard collision gate reacts to the current frame without temporal
    // delay, but does not receive the calibrated floor ripple.
    for (size_t index = 0U; index < points_buffer_.size(); ++index) {
      if (geometry_qualified_mark(
          index, ground_plane,
          recent_mark_ground_guard_height_m_,
          recent_mark_min_vertical_span_m_))
      {
        immediate_obstacle_points_buffer_.push_back(points_buffer_[index]);
      }
    }

    const auto append_confirmed_point =
      [this, &ground_plane](size_t index)
      {
        ++latest_temporally_confirmed_mark_points_;
        if (geometry_qualified_mark(
            index, ground_plane,
            recent_mark_ground_guard_height_m_,
            recent_mark_min_vertical_span_m_))
        {
          confirmed_sensor_points_buffer_.push_back(sensor_points_buffer_[index]);
        }
        if (geometry_qualified_mark(
            index, ground_plane,
            persistent_mark_ground_guard_height_m_,
            persistent_mark_min_vertical_span_m_))
        {
          persistent_sensor_points_buffer_.push_back(sensor_points_buffer_[index]);
        }
      };

    if (!persistent_mark_confirmation_enabled_ ||
      persistent_mark_confirmation_frames_ <= 1 ||
      !(voxel_size_ > 0.0))
    {
      std::fill(
        persistent_mark_counts_buffer_.begin(),
        persistent_mark_counts_buffer_.end(),
        static_cast<uint8_t>(std::min(persistent_mark_confirmation_frames_, 255)));
      for (size_t index = 0U; index < sensor_points_buffer_.size(); ++index) {
        append_confirmed_point(index);
      }
      return;
    }

    // First compute every count from previous generations. Updating the table
    // in this pass would let neighboring points from the same image count as
    // multiple temporal observations.
    for (size_t index = 0U; index < voxel_keys_buffer_.size(); ++index) {
      persistent_mark_counts_buffer_[index] =
        persistent_mark_confirmation_count(voxel_keys_buffer_[index]);
    }

    for (size_t index = 0U; index < voxel_keys_buffer_.size(); ++index) {
      const size_t key = voxel_keys_buffer_[index];
      persistent_mark_last_seen_generation_[key] = current_voxel_generation_;
      persistent_mark_hit_count_[key] = persistent_mark_counts_buffer_[index];
      if (persistent_mark_counts_buffer_[index] >=
        static_cast<uint8_t>(std::min(persistent_mark_confirmation_frames_, 255)))
      {
        append_confirmed_point(index);
      }
    }
  }

  bool spatially_consistent(
    const Image & image, int bpp, const RaySample & ray, float center_depth) const
  {
    if (!spatial_filter_enabled_ || spatial_min_neighbors_ <= 0) {
      return true;
    }
    const int offset = std::max(1, pixel_stride_);
    const double threshold = spatial_depth_threshold_m_ +
      spatial_depth_threshold_ratio_ * static_cast<double>(center_depth);
    int consistent = 0;
    static constexpr std::array<std::array<int, 2>, 8> offsets{{
      {{-1, -1}}, {{0, -1}}, {{1, -1}}, {{-1, 0}},
      {{1, 0}}, {{-1, 1}}, {{0, 1}}, {{1, 1}}
    }};
    for (const auto & direction : offsets) {
      const int u = static_cast<int>(ray.u) + direction[0] * offset;
      const int v = static_cast<int>(ray.v) + direction[1] * offset;
      if (u < 0 || v < 0 || u >= static_cast<int>(image.width) ||
        v >= static_cast<int>(image.height))
      {
        continue;
      }
      const uint8_t * ptr = image.data.data() +
        static_cast<size_t>(v) * image.step + static_cast<size_t>(u) * bpp;
      const float neighbor_depth = read_depth_m(image, ptr);
      if (std::isfinite(neighbor_depth) &&
        std::abs(static_cast<double>(neighbor_depth - center_depth)) <= threshold)
      {
        ++consistent;
        if (consistent >= spatial_min_neighbors_) {
          return true;
        }
      }
    }
    return false;
  }

  float temporally_stabilized_depth(size_t ray_index, float depth)
  {
    if (!temporal_filter_enabled_ || ray_index >= temporal_depth_buffer_.size()) {
      return depth;
    }
    const float previous = temporal_depth_buffer_[ray_index];
    float filtered = depth;
    if (std::isfinite(previous) &&
      std::abs(static_cast<double>(depth - previous)) <= temporal_max_delta_m_)
    {
      filtered = static_cast<float>(
        temporal_alpha_ * static_cast<double>(depth) +
        (1.0 - temporal_alpha_) * static_cast<double>(previous));
    }
    // Large changes are accepted immediately so dynamic obstacles do not
    // leave temporal ghosts in the local collision cloud.
    temporal_depth_buffer_[ray_index] = filtered;
    return filtered;
  }

  void advance_voxel_generation()
  {
    if (!(voxel_size_ > 0.0)) {
      return;
    }
    ++current_voxel_generation_;
    if (current_voxel_generation_ == 0U) {
      std::fill(voxel_generation_.begin(), voxel_generation_.end(), 0U);
      std::fill(
        persistent_mark_last_seen_generation_.begin(),
        persistent_mark_last_seen_generation_.end(), 0U);
      std::fill(
        persistent_mark_hit_count_.begin(),
        persistent_mark_hit_count_.end(), 0U);
      std::fill(column_generation_.begin(), column_generation_.end(), 0U);
      current_voxel_generation_ = 1U;
    }
  }

  void worker_loop()
  {
    steady_clock::time_point next_allowed_process{};
    while (!stop_worker_.load()) {
      PendingFrame pending;
      {
        std::unique_lock<std::mutex> lock(mailbox_mutex_);
        mailbox_cv_.wait(lock, [this]() {
          return stop_worker_.load() || latest_pending_.has_value();
        });
        if (stop_worker_.load()) {
          break;
        }
        pending = std::move(*latest_pending_);
        latest_pending_.reset();
      }

      if (max_rate_hz_ > 0.0) {
        const auto period = std::chrono::duration<double>(1.0 / max_rate_hz_);
        const auto now_steady = steady_clock::now();
        if (next_allowed_process.time_since_epoch().count() != 0 && now_steady < next_allowed_process) {
          std::unique_lock<std::mutex> lock(mailbox_mutex_);
          mailbox_cv_.wait_until(lock, next_allowed_process, [this]() {
            return stop_worker_.load() || latest_pending_.has_value();
          });
          if (stop_worker_.load()) {
            break;
          }
          if (latest_pending_.has_value()) {
            pending = std::move(*latest_pending_);
            latest_pending_.reset();
            ++rate_replaced_frames_;
          }
        }
        next_allowed_process = steady_clock::now() +
          std::chrono::duration_cast<steady_clock::duration>(period);
      }

      const double age_before_process = image_age_ms(pending.image->header.stamp);
      if (max_input_age_ms_ > 0.0 && age_before_process > max_input_age_ms_) {
        ++stale_dropped_frames_;
        continue;
      }
      process_frame(pending);
    }
  }

  static bool solve_ground_least_squares(
    const std::vector<Vec3f> & points, double candidate_min_z, double candidate_max_z,
    const GroundPlane * seed, double residual_limit, GroundPlane & result)
  {
    double matrix[3][4]{};
    size_t count = 0U;
    for (const auto & point : points) {
      if (point.z < candidate_min_z || point.z > candidate_max_z) {
        continue;
      }
      if (seed != nullptr) {
        const double residual = static_cast<double>(point.z) -
          (seed->a * point.x + seed->b * point.y + seed->c);
        if (std::abs(residual) > residual_limit) {
          continue;
        }
      }
      const double x = point.x;
      const double y = point.y;
      const double z = point.z;
      matrix[0][0] += x * x;
      matrix[0][1] += x * y;
      matrix[0][2] += x;
      matrix[0][3] += x * z;
      matrix[1][0] += x * y;
      matrix[1][1] += y * y;
      matrix[1][2] += y;
      matrix[1][3] += y * z;
      matrix[2][0] += x;
      matrix[2][1] += y;
      matrix[2][2] += 1.0;
      matrix[2][3] += z;
      ++count;
    }
    if (count < 3U) {
      return false;
    }

    for (int column = 0; column < 3; ++column) {
      int pivot = column;
      for (int row = column + 1; row < 3; ++row) {
        if (std::abs(matrix[row][column]) > std::abs(matrix[pivot][column])) {
          pivot = row;
        }
      }
      if (std::abs(matrix[pivot][column]) < 1.0e-9) {
        return false;
      }
      if (pivot != column) {
        for (int item = column; item < 4; ++item) {
          std::swap(matrix[column][item], matrix[pivot][item]);
        }
      }
      const double divisor = matrix[column][column];
      for (int item = column; item < 4; ++item) {
        matrix[column][item] /= divisor;
      }
      for (int row = 0; row < 3; ++row) {
        if (row == column) {
          continue;
        }
        const double factor = matrix[row][column];
        for (int item = column; item < 4; ++item) {
          matrix[row][item] -= factor * matrix[column][item];
        }
      }
    }
    result.a = matrix[0][3];
    result.b = matrix[1][3];
    result.c = matrix[2][3];
    result.inliers = count;
    return std::isfinite(result.a) && std::isfinite(result.b) && std::isfinite(result.c);
  }

  GroundPlane fit_ground_plane(const std::vector<Vec3f> & points)
  {
    GroundPlane seed = last_ground_plane_;
    if (!seed.valid) {
      seed.valid = true;
      seed.a = 0.0;
      seed.b = 0.0;
      seed.c = 0.0;
    }

    GroundPlane initial;
    for (const auto & point : points) {
      if (point.z >= ground_plane_candidate_min_z_ &&
        point.z <= ground_plane_candidate_max_z_ &&
        std::abs(
          static_cast<double>(point.z) -
          (seed.a * point.x + seed.b * point.y + seed.c)) <=
        ground_plane_seed_tolerance_)
      {
        ++initial.candidates;
      }
    }
    if (initial.candidates < static_cast<size_t>(ground_plane_min_inliers_) ||
      !solve_ground_least_squares(
        points, ground_plane_candidate_min_z_, ground_plane_candidate_max_z_,
        &seed, ground_plane_seed_tolerance_, initial))
    {
      return last_ground_plane_.valid ? last_ground_plane_ : initial;
    }

    GroundPlane refined;
    refined.candidates = initial.candidates;
    if (!solve_ground_least_squares(
        points, ground_plane_candidate_min_z_, ground_plane_candidate_max_z_,
        &initial, ground_plane_fit_tolerance_, refined))
    {
      return last_ground_plane_.valid ? last_ground_plane_ : refined;
    }
    const double inlier_ratio = static_cast<double>(refined.inliers) /
      static_cast<double>(std::max<size_t>(1U, refined.candidates));
    const double slope = std::hypot(refined.a, refined.b);
    refined.valid = refined.inliers >= static_cast<size_t>(ground_plane_min_inliers_) &&
      inlier_ratio >= ground_plane_min_inlier_ratio_ &&
      slope <= ground_plane_max_slope_ && std::abs(refined.c) <= 0.10;
    if (!refined.valid) {
      return last_ground_plane_.valid ? last_ground_plane_ : refined;
    }

    if (last_ground_plane_.valid) {
      const double slope_step = std::hypot(
        refined.a - last_ground_plane_.a,
        refined.b - last_ground_plane_.b);
      const double offset_step = std::abs(refined.c - last_ground_plane_.c);
      if (slope_step > ground_plane_max_slope_step_ ||
        offset_step > ground_plane_max_offset_step_)
      {
        return last_ground_plane_;
      }

      const double alpha = ground_plane_temporal_alpha_;
      refined.a = last_ground_plane_.a + alpha * (refined.a - last_ground_plane_.a);
      refined.b = last_ground_plane_.b + alpha * (refined.b - last_ground_plane_.b);
      refined.c = last_ground_plane_.c + alpha * (refined.c - last_ground_plane_.c);
    }
    last_ground_plane_ = refined;
    return refined;
  }

  bool is_ground_point(const Vec3f & point, const GroundPlane & plane) const
  {
    if (plane.valid) {
      const double residual = static_cast<double>(point.z) -
        (plane.a * point.x + plane.b * point.y + plane.c);
      return residual >= -ground_plane_remove_below_ &&
             residual <= ground_plane_remove_above_;
    }
    return point.z >= ground_z_min_ && point.z <= ground_z_max_;
  }

  double ground_residual(const Vec3f & point, const GroundPlane & plane) const
  {
    return static_cast<double>(point.z) -
      (plane.a * point.x + plane.b * point.y + plane.c);
  }

  void process_frame(const PendingFrame & pending)
  {
    const auto process_start = steady_clock::now();
    const Image & image = *pending.image;

    const int bpp = bytes_per_pixel(image.encoding);
    if (bpp == 0) {
      ++encoding_dropped_frames_;
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Unsupported depth encoding '%s'; expected 16UC1, mono16 or 32FC1",
        image.encoding.c_str());
      return;
    }
    if (image.width == 0U || image.height == 0U ||
      image.step < image.width * static_cast<uint32_t>(bpp) || image.data.empty())
    {
      ++invalid_image_dropped_frames_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Invalid depth image layout");
      return;
    }

    const auto camera_info_result = latest_camera_info();
    if (!camera_info_result.has_value()) {
      ++camera_info_dropped_frames_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for CameraInfo on %s", camera_info_topic_.c_str());
      return;
    }
    const CameraInfo & camera_info = camera_info_result->first;
    const uint64_t camera_info_version = camera_info_result->second;

    double tf_lookup_ms = 0.0;
    if (!ensure_cached_transform(image.header.frame_id, tf_lookup_ms)) {
      return;
    }
    if (!rebuild_ray_table_if_needed(image, camera_info, camera_info_version)) {
      ++camera_info_dropped_frames_;
      return;
    }

    advance_voxel_generation();
    points_buffer_.clear();
    sensor_points_buffer_.clear();
    voxel_keys_buffer_.clear();
    raw_points_buffer_.clear();
    raw_sensor_points_buffer_.clear();
    size_t valid_depth_pixels = 0U;

    for (size_t ray_index = 0U; ray_index < ray_table_.size(); ++ray_index) {
      const RaySample & ray = ray_table_[ray_index];
      const uint8_t * ptr = image.data.data() +
        static_cast<size_t>(ray.v) * image.step + static_cast<size_t>(ray.u) * bpp;
      float depth_m = read_depth_m(image, ptr);
      if (!std::isfinite(depth_m) || depth_m < min_range_ ||
        (max_range_ > 0.0 && depth_m > max_range_))
      {
        continue;
      }
      ++valid_depth_pixels;
      if (!spatially_consistent(image, bpp, ray, depth_m)) {
        continue;
      }
      depth_m = temporally_stabilized_depth(ray_index, depth_m);

      const Vec3f sensor_point{
        ray.ray_x * depth_m, ray.ray_y * depth_m, depth_m};
      const Vec3f point = cached_transform_.apply(
        sensor_point.x, sensor_point.y, sensor_point.z);
      if (!inside_crop(point)) {
        continue;
      }
      if (remove_self_ && inside_self(point)) {
        continue;
      }
      raw_points_buffer_.push_back(point);
      raw_sensor_points_buffer_.push_back(sensor_point);
    }

    GroundPlane ground_plane;
    if (ground_filter_enabled_ && adaptive_ground_plane_) {
      ground_plane = fit_ground_plane(raw_points_buffer_);
    }
    size_t ground_removed = 0U;
    for (size_t index = 0U; index < raw_points_buffer_.size(); ++index) {
      const Vec3f & point = raw_points_buffer_[index];
      if (ground_filter_enabled_ && is_ground_point(point, ground_plane)) {
        ++ground_removed;
        continue;
      }
      const auto voxel_key = mark_voxel_first_hit(point);
      if (!voxel_key.has_value()) {
        continue;
      }
      points_buffer_.push_back(point);
      sensor_points_buffer_.push_back(raw_sensor_points_buffer_[index]);
      voxel_keys_buffer_.push_back(*voxel_key);
    }

    latest_ground_plane_valid_.store(ground_plane.valid);
    latest_ground_plane_a_.store(ground_plane.a);
    latest_ground_plane_b_.store(ground_plane.b);
    latest_ground_plane_c_.store(ground_plane.c);
    latest_ground_plane_candidates_.store(ground_plane.candidates);
    latest_ground_plane_inliers_.store(ground_plane.inliers);
    latest_ground_removed_.store(ground_removed);
    latest_ground_speckles_removed_.store(0U);

    if (voxel_outlier_filter_enabled_ && voxel_min_neighbors_ > 0 &&
      voxel_size_ > 0.0 && !points_buffer_.empty())
    {
      size_t write_index = 0U;
      size_t ground_speckles_removed = 0U;
      for (size_t read_index = 0U; read_index < points_buffer_.size(); ++read_index) {
        const int neighbors = occupied_voxel_neighbors(voxel_keys_buffer_[read_index]);
        if (neighbors < voxel_min_neighbors_) {
          continue;
        }
        if (ground_filter_enabled_) {
          const double residual = ground_residual(points_buffer_[read_index], ground_plane);
          const double speckle_lower_bound = ground_plane.valid ?
            ground_plane_remove_above_ : ground_z_max_;
          if (residual > speckle_lower_bound &&
            residual <= ground_speckle_max_height_ &&
            neighbors < ground_speckle_min_neighbors_)
          {
            ++ground_speckles_removed;
            continue;
          }
        }
        if (write_index != read_index) {
          points_buffer_[write_index] = points_buffer_[read_index];
          sensor_points_buffer_[write_index] = sensor_points_buffer_[read_index];
          voxel_keys_buffer_[write_index] = voxel_keys_buffer_[read_index];
        }
        ++write_index;
      }
      points_buffer_.resize(write_index);
      sensor_points_buffer_.resize(write_index);
      voxel_keys_buffer_.resize(write_index);
      latest_ground_speckles_removed_.store(ground_speckles_removed);
    }

    // An empty filtered frame is still a valid observation. Publishing it
    // keeps the watchdog alive and lets STVL clear an obstacle after the
    // camera sees that space become free.
    if (points_buffer_.empty()) {
      ++empty_published_frames_;
    }

    build_persistent_mark_cloud(ground_plane);

    auto cloud = make_xyz_cloud(image.header, output_frame_, points_buffer_);
    cloud_pub_->publish(cloud);
    auto immediate_obstacle_cloud = make_xyz_cloud(
      image.header, output_frame_, immediate_obstacle_points_buffer_);
    clear_sensor_points_buffer_.clear();
    for (size_t index = 0U; index < raw_sensor_points_buffer_.size();
      index += kClearRaySubsample)
    {
      clear_sensor_points_buffer_.push_back(raw_sensor_points_buffer_[index]);
    }
    auto raw_clear_sensor_cloud = make_xyz_cloud(
      image.header, image.header.frame_id, clear_sensor_points_buffer_);
    auto confirmed_sensor_cloud = make_xyz_cloud(
      image.header, image.header.frame_id, confirmed_sensor_points_buffer_);
    auto persistent_sensor_cloud = make_xyz_cloud(
      image.header, image.header.frame_id, persistent_sensor_points_buffer_);
    // Marking, collision checking and clearing use separate topics. Raw valid
    // depth rays remain in the clearing cloud so reversing can erase stale
    // voxels even when the endpoint itself is classified as floor.
    sensor_cloud_pub_->publish(confirmed_sensor_cloud);
    persistent_sensor_cloud_pub_->publish(persistent_sensor_cloud);
    immediate_obstacle_cloud_pub_->publish(immediate_obstacle_cloud);
    const double valid_depth_ratio = ray_table_.empty() ? 0.0 :
      static_cast<double>(valid_depth_pixels) / static_cast<double>(ray_table_.size());
    if (valid_depth_ratio >= min_clear_valid_depth_ratio_) {
      clear_sensor_cloud_pub_->publish(raw_clear_sensor_cloud);
      ++clear_qualified_frames_;
    } else {
      ++clear_suppressed_frames_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Suppressing STVL clearing: valid depth ratio %.3f < %.3f",
        valid_depth_ratio, min_clear_valid_depth_ratio_);
    }
    ++published_frames_;

    const auto process_end = steady_clock::now();
    const double process_ms = duration_ms(process_start, process_end);
    const double output_age_ms = image_age_ms(image.header.stamp);
    double output_gap_ms = -1.0;
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      output_gap_ms = steady_gap_ms(last_output_steady_, process_end);
      last_output_steady_ = process_end;
      process_window_->add(process_ms);
      age_window_->add(output_age_ms);
      if (output_gap_ms >= 0.0) {
        output_gap_window_->add(output_gap_ms);
        latest_output_gap_ms_ = output_gap_ms;
        if (output_gap_ms > stall_warn_gap_ms_) {
          ++output_stall_events_;
        }
      }
      latest_process_ms_ = process_ms;
      latest_output_age_ms_ = output_age_ms;
      latest_tf_lookup_ms_ = tf_lookup_ms;
      latest_sampled_pixels_ = ray_table_.size();
      latest_valid_depth_pixels_ = valid_depth_pixels;
      latest_output_points_ = points_buffer_.size();
      latest_immediate_obstacle_points_ = immediate_obstacle_points_buffer_.size();
      latest_clear_sensor_points_ = clear_sensor_points_buffer_.size();
      latest_confirmed_mark_points_ = confirmed_sensor_points_buffer_.size();
      latest_persistent_mark_points_ = persistent_sensor_points_buffer_.size();
    }

    if (process_ms > process_warn_ms_ || output_age_ms > age_warn_ms_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "STEP10V2.1 latency spike: process=%.1fms age=%.1fms output_gap=%.1fms",
        process_ms, output_age_ms, output_gap_ms);
    }
  }

  void publish_stats()
  {
    MetricWindow::Snapshot process;
    MetricWindow::Snapshot age;
    MetricWindow::Snapshot input_gap;
    MetricWindow::Snapshot arrival_gap;
    MetricWindow::Snapshot output_gap;
    double latest_process = 0.0;
    double latest_age = 0.0;
    double latest_input_age = 0.0;
    double latest_tf = 0.0;
    double latest_message_gap = 0.0;
    double latest_arrival_gap = 0.0;
    double latest_output_gap = 0.0;
    size_t sampled_pixels = 0U;
    size_t valid_pixels = 0U;
    size_t output_points = 0U;
    size_t immediate_obstacle_points = 0U;
    size_t clear_sensor_points = 0U;
    size_t recent_mark_points = 0U;
    size_t persistent_mark_points = 0U;
    uint32_t image_width = 0U;
    uint32_t image_height = 0U;
    std::string encoding;
    std::string source_frame;
    steady_clock::time_point last_input_arrival;
    steady_clock::time_point last_output;

    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      process = process_window_->snapshot();
      age = age_window_->snapshot();
      input_gap = input_gap_window_->snapshot();
      arrival_gap = arrival_gap_window_->snapshot();
      output_gap = output_gap_window_->snapshot();
      latest_process = latest_process_ms_;
      latest_age = latest_output_age_ms_;
      latest_input_age = latest_input_age_ms_;
      latest_tf = latest_tf_lookup_ms_;
      latest_message_gap = latest_input_gap_ms_;
      latest_arrival_gap = latest_arrival_gap_ms_;
      latest_output_gap = latest_output_gap_ms_;
      sampled_pixels = latest_sampled_pixels_;
      valid_pixels = latest_valid_depth_pixels_;
      output_points = latest_output_points_;
      immediate_obstacle_points = latest_immediate_obstacle_points_;
      clear_sensor_points = latest_clear_sensor_points_;
      recent_mark_points = latest_confirmed_mark_points_;
      persistent_mark_points = latest_persistent_mark_points_;
      image_width = latest_image_width_;
      image_height = latest_image_height_;
      encoding = latest_encoding_;
      source_frame = latest_source_frame_;
      last_input_arrival = last_input_arrival_steady_;
      last_output = last_output_steady_;
    }

    const auto now_steady = steady_clock::now();
    const double since_input_ms = steady_gap_ms(last_input_arrival, now_steady);
    const double since_output_ms = steady_gap_ms(last_output, now_steady);
    if (since_input_ms > stall_warn_gap_ms_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Depth input stalled for %.1f ms", since_input_ms);
    }
    if (published_frames_.load() > 0U && since_output_ms > stall_warn_gap_ms_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Local cloud output stalled for %.1f ms", since_output_ms);
    }

    const double input_hz = input_gap.average > 0.0 ? 1000.0 / input_gap.average : 0.0;
    const double arrival_hz = arrival_gap.average > 0.0 ? 1000.0 / arrival_gap.average : 0.0;
    const double output_hz = output_gap.average > 0.0 ? 1000.0 / output_gap.average : 0.0;
    const uint64_t known_dropped = mailbox_replaced_frames_.load() +
      rate_replaced_frames_.load() + stale_dropped_frames_.load() +
      tf_dropped_frames_.load() + camera_info_dropped_frames_.load() +
      encoding_dropped_frames_.load() + invalid_image_dropped_frames_.load();

    std::ostringstream json;
    json.setf(std::ios::fixed);
    json.precision(2);
    json << "{"
         << "\"version\":\"STEP10V2.1\","
         << "\"image_width\":" << image_width << ","
         << "\"image_height\":" << image_height << ","
         << "\"sampled_pixels\":" << sampled_pixels << ","
         << "\"valid_depth_pixels\":" << valid_pixels << ","
         << "\"output_points\":" << output_points << ","
         << "\"immediate_obstacle_points\":" << immediate_obstacle_points << ","
         << "\"clear_sensor_points\":" << clear_sensor_points << ","
         << "\"temporally_confirmed_points\":" <<
              latest_temporally_confirmed_mark_points_.load() << ","
         << "\"recent_mark_points\":" << recent_mark_points << ","
         << "\"persistent_mark_points\":" << persistent_mark_points << ","
         << "\"persistent_mark_confirmation_enabled\":" <<
              (persistent_mark_confirmation_enabled_ ? "true" : "false") << ","
         << "\"persistent_mark_confirmation_frames\":" <<
              persistent_mark_confirmation_frames_ << ","
         << "\"persistent_geometry_guard_enabled\":" <<
              (persistent_geometry_guard_enabled_ ? "true" : "false") << ","
         << "\"recent_mark_ground_guard_height_m\":" <<
              recent_mark_ground_guard_height_m_ << ","
         << "\"recent_mark_min_vertical_span_m\":" <<
              recent_mark_min_vertical_span_m_ << ","
         << "\"persistent_mark_ground_guard_height_m\":" <<
              persistent_mark_ground_guard_height_m_ << ","
         << "\"persistent_mark_min_vertical_span_m\":" <<
              persistent_mark_min_vertical_span_m_ << ","
         << "\"input_age_ms\":" << latest_input_age << ","
         << "\"output_age_ms\":" << latest_age << ","
         << "\"process_ms\":" << latest_process << ","
         << "\"tf_lookup_ms\":" << latest_tf << ","
         << "\"input_gap_ms\":" << latest_message_gap << ","
         << "\"arrival_gap_ms\":" << latest_arrival_gap << ","
         << "\"output_gap_ms\":" << latest_output_gap << ","
         << "\"process_avg_ms\":" << process.average << ","
         << "\"process_p95_ms\":" << process.p95 << ","
         << "\"process_max_ms\":" << process.maximum << ","
         << "\"age_avg_ms\":" << age.average << ","
         << "\"age_p95_ms\":" << age.p95 << ","
         << "\"age_max_ms\":" << age.maximum << ","
         << "\"input_gap_avg_ms\":" << input_gap.average << ","
         << "\"input_gap_p95_ms\":" << input_gap.p95 << ","
         << "\"input_gap_max_ms\":" << input_gap.maximum << ","
         << "\"arrival_gap_max_ms\":" << arrival_gap.maximum << ","
         << "\"output_gap_avg_ms\":" << output_gap.average << ","
         << "\"output_gap_p95_ms\":" << output_gap.p95 << ","
         << "\"output_gap_max_ms\":" << output_gap.maximum << ","
         << "\"input_hz\":" << input_hz << ","
         << "\"arrival_hz\":" << arrival_hz << ","
         << "\"output_hz\":" << output_hz << ","
         << "\"ms_since_last_input\":" << since_input_ms << ","
         << "\"ms_since_last_output\":" << since_output_ms << ","
         << "\"received_frames\":" << received_frames_.load() << ","
         << "\"published_frames\":" << published_frames_.load() << ","
         << "\"empty_published_frames\":" << empty_published_frames_.load() << ","
         << "\"clear_qualified_frames\":" << clear_qualified_frames_.load() << ","
         << "\"clear_suppressed_frames\":" << clear_suppressed_frames_.load() << ","
         << "\"min_clear_valid_depth_ratio\":" << min_clear_valid_depth_ratio_ << ","
         << "\"known_dropped_frames\":" << known_dropped << ","
         << "\"mailbox_replaced_frames\":" << mailbox_replaced_frames_.load() << ","
         << "\"rate_replaced_frames\":" << rate_replaced_frames_.load() << ","
         << "\"stale_dropped_frames\":" << stale_dropped_frames_.load() << ","
         << "\"tf_dropped_frames\":" << tf_dropped_frames_.load() << ","
         << "\"duplicate_timestamp_frames\":" << duplicate_timestamp_frames_.load() << ","
         << "\"backward_timestamp_frames\":" << backward_timestamp_frames_.load() << ","
         << "\"input_stall_events\":" << input_stall_events_.load() << ","
         << "\"output_stall_events\":" << output_stall_events_.load() << ","
         << "\"tf_cache_refreshes\":" << tf_cache_refreshes_.load() << ","
         << "\"ray_table_rebuilds\":" << ray_table_rebuilds_.load() << ","
         << "\"pixel_stride\":" << pixel_stride_ << ","
         << "\"voxel_size_m\":" << voxel_size_ << ","
         << "\"ground_plane_enabled\":" <<
              (adaptive_ground_plane_ ? "true" : "false") << ","
         << "\"ground_plane_valid\":" <<
              (latest_ground_plane_valid_.load() ? "true" : "false") << ","
         << "\"ground_plane_a\":" << latest_ground_plane_a_.load() << ","
         << "\"ground_plane_b\":" << latest_ground_plane_b_.load() << ","
         << "\"ground_plane_c\":" << latest_ground_plane_c_.load() << ","
         << "\"ground_plane_candidates\":" << latest_ground_plane_candidates_.load() << ","
         << "\"ground_plane_inliers\":" << latest_ground_plane_inliers_.load() << ","
         << "\"ground_removed_points\":" << latest_ground_removed_.load() << ","
         << "\"ground_speckles_removed\":" << latest_ground_speckles_removed_.load() << ","
         << "\"intrinsics_scaled\":" << (ray_intrinsics_scaled_.load() ? "true" : "false") << ","
         << "\"encoding\":\"" << encoding << "\","
         << "\"source_frame\":\"" << source_frame << "\","
         << "\"output_frame\":\"" << output_frame_ << "\""
         << "}";

    std_msgs::msg::String stats;
    stats.data = json.str();
    stats_pub_->publish(stats);

    RCLCPP_INFO(
      get_logger(),
      "STEP10V2.1 %ux%u -> %zu live / %zu temporal / %zu recent / %zu persistent pts | "
      "process avg/p95/max %.1f/%.1f/%.1f ms | "
      "age avg/p95/max %.1f/%.1f/%.1f ms | in/out %.1f/%.1f Hz | gap max %.1f ms | "
      "ground valid=%s plane=(%.4f,%.4f,%.4f) inliers=%zu/%zu removed=%zu+%zu "
      "immediate=%zu clear_rays=%zu",
      image_width, image_height, output_points,
      latest_temporally_confirmed_mark_points_.load(),
      recent_mark_points, persistent_mark_points,
      process.average, process.p95, process.maximum,
      age.average, age.p95, age.maximum,
      input_hz, output_hz, output_gap.maximum,
      latest_ground_plane_valid_.load() ? "true" : "false",
      latest_ground_plane_a_.load(), latest_ground_plane_b_.load(), latest_ground_plane_c_.load(),
      latest_ground_plane_inliers_.load(), latest_ground_plane_candidates_.load(),
      latest_ground_removed_.load(), latest_ground_speckles_removed_.load(),
      immediate_obstacle_points, clear_sensor_points);
  }

  void publish_markers()
  {
    if (!publish_markers_) {
      return;
    }
    const auto stamp = now();
    MarkerArray array;
    array.markers.push_back(cube_marker(
      0, "step10v21_crop", output_frame_, stamp,
      {x_min_, y_min_, z_min_}, {x_max_, y_max_, z_max_},
      {0.10F, 0.75F, 1.00F, 0.055F}));
    if (remove_self_) {
      array.markers.push_back(cube_marker(
        1, "step10v21_self", output_frame_, stamp,
        {self_x_min_, self_y_min_, self_z_min_},
        {self_x_max_, self_y_max_, self_z_max_},
        {1.00F, 0.20F, 0.15F, 0.15F}));
    }
    if (ground_filter_enabled_) {
      array.markers.push_back(cube_marker(
        2, "step10v21_ground", output_frame_, stamp,
        {x_min_, y_min_, ground_z_min_}, {x_max_, y_max_, ground_z_max_},
        {1.00F, 0.75F, 0.10F, 0.12F}));
    }
    marker_pub_->publish(array);
  }

  std::string depth_topic_;
  std::string pipeline_version_;
  std::string camera_info_topic_;
  std::string output_topic_;
  std::string sensor_output_topic_;
  std::string persistent_sensor_output_topic_;
  std::string immediate_obstacle_output_topic_;
  std::string clear_sensor_output_topic_;
  std::string stats_topic_;
  std::string marker_topic_;
  std::string output_frame_;

  double max_rate_hz_{30.0};
  int pixel_stride_{2};
  double depth_unit_scale_{0.001};
  double min_range_{0.20};
  double max_range_{4.0};
  double voxel_size_{0.03};
  bool spatial_filter_enabled_{true};
  double spatial_depth_threshold_m_{0.08};
  double spatial_depth_threshold_ratio_{0.025};
  int spatial_min_neighbors_{2};
  bool temporal_filter_enabled_{true};
  double temporal_alpha_{0.65};
  double temporal_max_delta_m_{0.06};
  bool voxel_outlier_filter_enabled_{true};
  int voxel_min_neighbors_{1};
  bool persistent_mark_confirmation_enabled_{true};
  int persistent_mark_confirmation_frames_{3};
  int persistent_mark_max_gap_frames_{1};
  int persistent_mark_neighbor_radius_{1};
  bool persistent_geometry_guard_enabled_{true};
  double recent_mark_ground_guard_height_m_{0.12};
  double recent_mark_min_vertical_span_m_{0.025};
  double persistent_mark_ground_guard_height_m_{0.15};
  double persistent_mark_min_vertical_span_m_{0.04};
  int mark_geometry_neighbor_radius_{1};
  double transform_timeout_sec_{0.50};
  double max_input_age_ms_{150.0};
  double min_clear_valid_depth_ratio_{0.05};
  int roi_u_min_{0};
  int roi_u_max_{-1};
  int roi_v_min_{0};
  int roi_v_max_{-1};

  double x_min_{0.15};
  double x_max_{4.00};
  double y_min_{-2.50};
  double y_max_{2.50};
  double z_min_{-0.50};
  double z_max_{2.00};

  bool remove_self_{true};
  double self_x_min_{-0.36};
  double self_x_max_{0.36};
  double self_y_min_{-0.36};
  double self_y_max_{0.36};
  double self_z_min_{-0.10};
  double self_z_max_{0.90};

  bool ground_filter_enabled_{false};
  double ground_z_min_{-0.06};
  double ground_z_max_{0.08};
  bool adaptive_ground_plane_{true};
  double ground_plane_candidate_min_z_{-0.15};
  double ground_plane_candidate_max_z_{0.12};
  double ground_plane_fit_tolerance_{0.025};
  double ground_plane_seed_tolerance_{0.06};
  double ground_plane_temporal_alpha_{0.18};
  double ground_plane_max_slope_step_{0.02};
  double ground_plane_max_offset_step_{0.02};
  double ground_plane_remove_below_{0.035};
  double ground_plane_remove_above_{0.025};
  double ground_plane_max_slope_{0.12};
  int ground_plane_min_inliers_{120};
  double ground_plane_min_inlier_ratio_{0.30};
  double ground_speckle_max_height_{0.06};
  int ground_speckle_min_neighbors_{4};
  bool publish_markers_{true};
  double stats_period_sec_{1.0};
  int stats_window_size_{300};
  double process_warn_ms_{50.0};
  double age_warn_ms_{120.0};
  double stall_warn_gap_ms_{120.0};

  mutable std::mutex camera_info_mutex_;
  std::optional<CameraInfo> latest_camera_info_;
  uint64_t camera_info_version_{0U};

  std::mutex mailbox_mutex_;
  std::condition_variable mailbox_cv_;
  std::optional<PendingFrame> latest_pending_;
  std::atomic<bool> stop_worker_{false};
  std::thread worker_thread_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  RigidTransform cached_transform_{};
  std::string cached_transform_source_frame_;
  bool transform_cached_{false};

  std::vector<RaySample> ray_table_;
  std::vector<Vec3f> points_buffer_;
  std::vector<Vec3f> sensor_points_buffer_;
  std::vector<Vec3f> immediate_obstacle_points_buffer_;
  std::vector<Vec3f> confirmed_sensor_points_buffer_;
  std::vector<Vec3f> persistent_sensor_points_buffer_;
  std::vector<Vec3f> raw_points_buffer_;
  std::vector<Vec3f> raw_sensor_points_buffer_;
  std::vector<Vec3f> clear_sensor_points_buffer_;
  std::vector<size_t> voxel_keys_buffer_;
  std::vector<uint8_t> persistent_mark_counts_buffer_;
  std::vector<float> temporal_depth_buffer_;
  GroundPlane last_ground_plane_;
  bool ray_table_ready_{false};
  uint32_t ray_image_width_{0U};
  uint32_t ray_image_height_{0U};
  uint64_t ray_camera_info_version_{0U};
  std::atomic<bool> ray_intrinsics_scaled_{false};
  std::atomic<bool> latest_ground_plane_valid_{false};
  std::atomic<double> latest_ground_plane_a_{0.0};
  std::atomic<double> latest_ground_plane_b_{0.0};
  std::atomic<double> latest_ground_plane_c_{0.0};
  std::atomic<size_t> latest_ground_plane_candidates_{0U};
  std::atomic<size_t> latest_ground_plane_inliers_{0U};
  std::atomic<size_t> latest_ground_removed_{0U};
  std::atomic<size_t> latest_ground_speckles_removed_{0U};

  int64_t voxel_nx_{0};
  int64_t voxel_ny_{0};
  int64_t voxel_nz_{0};
  std::vector<uint32_t> voxel_generation_;
  std::vector<uint32_t> persistent_mark_last_seen_generation_;
  std::vector<uint8_t> persistent_mark_hit_count_;
  std::vector<uint32_t> column_generation_;
  std::vector<float> column_min_residual_;
  std::vector<float> column_max_residual_;
  uint32_t current_voxel_generation_{1U};

  rclcpp::Subscription<Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr sensor_cloud_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr persistent_sensor_cloud_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr immediate_obstacle_cloud_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr clear_sensor_cloud_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stats_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr marker_timer_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  mutable std::mutex stats_mutex_;
  std::unique_ptr<MetricWindow> process_window_;
  std::unique_ptr<MetricWindow> age_window_;
  std::unique_ptr<MetricWindow> input_gap_window_;
  std::unique_ptr<MetricWindow> arrival_gap_window_;
  std::unique_ptr<MetricWindow> output_gap_window_;

  steady_clock::time_point last_input_arrival_steady_{};
  steady_clock::time_point last_output_steady_{};
  int64_t last_input_stamp_ns_{0};

  double latest_process_ms_{0.0};
  double latest_output_age_ms_{0.0};
  double latest_input_age_ms_{0.0};
  double latest_tf_lookup_ms_{0.0};
  double latest_input_gap_ms_{0.0};
  double latest_arrival_gap_ms_{0.0};
  double latest_output_gap_ms_{0.0};
  size_t latest_sampled_pixels_{0U};
  size_t latest_valid_depth_pixels_{0U};
  size_t latest_output_points_{0U};
  size_t latest_immediate_obstacle_points_{0U};
  size_t latest_clear_sensor_points_{0U};
  size_t latest_confirmed_mark_points_{0U};
  size_t latest_persistent_mark_points_{0U};
  uint32_t latest_image_width_{0U};
  uint32_t latest_image_height_{0U};
  std::string latest_encoding_;
  std::string latest_source_frame_;

  std::atomic<uint64_t> received_frames_{0U};
  std::atomic<uint64_t> published_frames_{0U};
  std::atomic<uint64_t> mailbox_replaced_frames_{0U};
  std::atomic<uint64_t> rate_replaced_frames_{0U};
  std::atomic<uint64_t> stale_dropped_frames_{0U};
  std::atomic<uint64_t> tf_dropped_frames_{0U};
  std::atomic<uint64_t> camera_info_dropped_frames_{0U};
  std::atomic<uint64_t> encoding_dropped_frames_{0U};
  std::atomic<uint64_t> invalid_image_dropped_frames_{0U};
  std::atomic<uint64_t> empty_published_frames_{0U};
  std::atomic<uint64_t> clear_qualified_frames_{0U};
  std::atomic<uint64_t> clear_suppressed_frames_{0U};
  std::atomic<uint64_t> duplicate_timestamp_frames_{0U};
  std::atomic<uint64_t> backward_timestamp_frames_{0U};
  std::atomic<uint64_t> input_stall_events_{0U};
  std::atomic<uint64_t> output_stall_events_{0U};
  std::atomic<uint64_t> tf_cache_refreshes_{0U};
  std::atomic<uint64_t> ray_table_rebuilds_{0U};
  std::atomic<size_t> latest_temporally_confirmed_mark_points_{0U};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthImageToLocalCloudNode>());
  rclcpp::shutdown();
  return 0;
}
