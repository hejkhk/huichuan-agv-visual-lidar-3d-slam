#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
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

double ema(double previous, double sample, double alpha = 0.20)
{
  if (!(sample > 0.0) || !std::isfinite(sample)) {
    return previous;
  }
  if (!(previous > 0.0)) {
    return sample;
  }
  return previous + alpha * (sample - previous);
}

struct Vec3f
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
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
  {
    const auto dur = rclcpp::Duration::from_seconds(1.5);
    marker.lifetime.sec = static_cast<int32_t>(dur.nanoseconds() / 1000000000LL);
    marker.lifetime.nanosec = static_cast<uint32_t>(dur.nanoseconds() % 1000000000LL);
  }
  return marker;
}
}  // namespace

class DepthImageToLocalCloudNode final : public rclcpp::Node
{
public:
  DepthImageToLocalCloudNode()
  : Node("depth_image_to_local_cloud"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
    camera_info_topic_ = declare_parameter<std::string>(
      "camera_info_topic", "/camera/depth/camera_info");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/local_highres_cloud_v2");
    stats_topic_ = declare_parameter<std::string>(
      "stats_topic", "/local_highres_cloud_v2/stats");
    marker_topic_ = declare_parameter<std::string>(
      "marker_topic", "/local_highres_cloud_v2/crop_markers");
    output_frame_ = declare_parameter<std::string>("output_frame", "base_link");

    max_rate_hz_ = declare_parameter<double>("max_rate_hz", 30.0);
    pixel_stride_ = std::max(1, static_cast<int>(declare_parameter<int>("pixel_stride", 2)));
    depth_unit_scale_ = declare_parameter<double>("depth_unit_scale", 0.001);
    min_range_ = declare_parameter<double>("min_range", 0.20);
    max_range_ = declare_parameter<double>("max_range", 4.0);
    voxel_size_ = declare_parameter<double>("voxel_size", 0.03);
    transform_timeout_sec_ = declare_parameter<double>("transform_timeout", 0.015);

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
    publish_markers_ = declare_parameter<bool>("publish_markers", true);
    stats_period_sec_ = declare_parameter<double>("stats_period_sec", 1.0);

    validate_parameters();

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    const auto marker_qos = rclcpp::QoS(1).reliable().transient_local();

    cloud_pub_ = create_publisher<PointCloud2>(output_topic_, sensor_qos);
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

    RCLCPP_INFO(
      get_logger(),
      "STEP10V2 direct depth-image cloud started: %s + %s -> %s, frame=%s, "
      "stride=%d, voxel=%.3fm, max_rate=%.1fHz",
      depth_topic_.c_str(), camera_info_topic_.c_str(), output_topic_.c_str(),
      output_frame_.c_str(), pixel_stride_, voxel_size_, max_rate_hz_);
  }

private:
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
    if (!(depth_unit_scale_ > 0.0)) {
      throw std::invalid_argument("depth_unit_scale must be positive");
    }
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
    latest_camera_info_ = *msg;
  }

  std::optional<CameraInfo> latest_camera_info() const
  {
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    return latest_camera_info_;
  }

  double image_age_ms(const builtin_interfaces::msg::Time & stamp) const
  {
    if (stamp.sec == 0 && stamp.nanosec == 0) {
      return -1.0;
    }
    const rclcpp::Time image_time(stamp, get_clock()->get_clock_type());
    return (now() - image_time).seconds() * 1000.0;
  }

  std::optional<std::pair<RigidTransform, double>> lookup_transform(const Image & image)
  {
    if (image.header.frame_id.empty() || image.header.frame_id == output_frame_) {
      return std::make_pair(RigidTransform{}, 0.0);
    }

    const auto start = steady_clock::now();
    try {
      const auto transform = tf_buffer_.lookupTransform(
        output_frame_, image.header.frame_id, rclcpp::Time(image.header.stamp),
        rclcpp::Duration::from_seconds(transform_timeout_sec_));
      return std::make_pair(
        transform_from_msg(transform), duration_ms(start, steady_clock::now()));
    } catch (const tf2::TransformException & error) {
      ++tf_dropped_frames_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for TF %s <- %s: %s", output_frame_.c_str(),
        image.header.frame_id.c_str(), error.what());
      return std::nullopt;
    }
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

  int64_t voxel_key(const Vec3f & point, int64_t nx, int64_t ny) const
  {
    const int64_t ix = static_cast<int64_t>(std::floor((point.x - x_min_) / voxel_size_));
    const int64_t iy = static_cast<int64_t>(std::floor((point.y - y_min_) / voxel_size_));
    const int64_t iz = static_cast<int64_t>(std::floor((point.z - z_min_) / voxel_size_));
    return ix + nx * (iy + ny * iz);
  }

  void depth_callback(const Image::SharedPtr image)
  {
    const auto callback_start = steady_clock::now();
    const double arrival_age_ms = image_age_ms(image->header.stamp);
    ++received_frames_;

    if (last_input_time_.time_since_epoch().count() != 0) {
      const double dt = std::chrono::duration<double>(callback_start - last_input_time_).count();
      if (dt > 1.0e-6) {
        input_hz_ = ema(input_hz_, 1.0 / dt);
      }
    }
    last_input_time_ = callback_start;

    if (max_rate_hz_ > 0.0 && last_process_time_.time_since_epoch().count() != 0) {
      const double dt = std::chrono::duration<double>(callback_start - last_process_time_).count();
      if (dt < 1.0 / max_rate_hz_) {
        ++rate_dropped_frames_;
        return;
      }
    }
    last_process_time_ = callback_start;

    const int bpp = bytes_per_pixel(image->encoding);
    if (bpp == 0) {
      ++encoding_dropped_frames_;
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Unsupported depth encoding '%s'; expected 16UC1, mono16 or 32FC1",
        image->encoding.c_str());
      return;
    }
    if (image->width == 0U || image->height == 0U ||
      image->step < image->width * static_cast<uint32_t>(bpp) || image->data.empty())
    {
      ++invalid_image_dropped_frames_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Invalid depth image layout");
      return;
    }

    const auto camera_info = latest_camera_info();
    if (!camera_info.has_value()) {
      ++camera_info_dropped_frames_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for CameraInfo on %s", camera_info_topic_.c_str());
      return;
    }

    auto transform_result = lookup_transform(*image);
    if (!transform_result.has_value()) {
      return;
    }
    const RigidTransform & transform = transform_result->first;
    const double tf_wait_ms = transform_result->second;

    const double sx = camera_info->width > 0U ?
      static_cast<double>(image->width) / static_cast<double>(camera_info->width) : 1.0;
    const double sy = camera_info->height > 0U ?
      static_cast<double>(image->height) / static_cast<double>(camera_info->height) : 1.0;
    const double fx = camera_info->k[0] * sx;
    const double fy = camera_info->k[4] * sy;
    const double cx = camera_info->k[2] * sx;
    const double cy = camera_info->k[5] * sy;
    if (!(fx > 0.0 && fy > 0.0)) {
      ++camera_info_dropped_frames_;
      return;
    }

    const int u_begin = std::clamp(roi_u_min_, 0, static_cast<int>(image->width) - 1);
    const int u_end = roi_u_max_ < 0 ? static_cast<int>(image->width) :
      std::clamp(roi_u_max_, u_begin + 1, static_cast<int>(image->width));
    const int v_begin = std::clamp(roi_v_min_, 0, static_cast<int>(image->height) - 1);
    const int v_end = roi_v_max_ < 0 ? static_cast<int>(image->height) :
      std::clamp(roi_v_max_, v_begin + 1, static_cast<int>(image->height));

    const size_t sampled_capacity =
      static_cast<size_t>((u_end - u_begin + pixel_stride_ - 1) / pixel_stride_) *
      static_cast<size_t>((v_end - v_begin + pixel_stride_ - 1) / pixel_stride_);
    std::vector<Vec3f> points;
    points.reserve(sampled_capacity / 2U + 1024U);

    std::unordered_set<int64_t> occupied_voxels;
    int64_t nx = 1;
    int64_t ny = 1;
    if (voxel_size_ > 0.0) {
      nx = std::max<int64_t>(
        1, static_cast<int64_t>(std::ceil((x_max_ - x_min_) / voxel_size_)) + 1);
      ny = std::max<int64_t>(
        1, static_cast<int64_t>(std::ceil((y_max_ - y_min_) / voxel_size_)) + 1);
      occupied_voxels.reserve(sampled_capacity / 4U + 1024U);
    }

    size_t sampled_pixels = 0;
    size_t valid_depth_pixels = 0;
    for (int v = v_begin; v < v_end; v += pixel_stride_) {
      const uint8_t * row = image->data.data() + static_cast<size_t>(v) * image->step;
      for (int u = u_begin; u < u_end; u += pixel_stride_) {
        ++sampled_pixels;
        const float depth_m = read_depth_m(*image, row + static_cast<size_t>(u) * bpp);
        if (!std::isfinite(depth_m) || depth_m < min_range_ ||
          (max_range_ > 0.0 && depth_m > max_range_))
        {
          continue;
        }
        ++valid_depth_pixels;

        // Optical frame convention: x right, y down, z forward.
        const float optical_x = static_cast<float>((static_cast<double>(u) - cx) * depth_m / fx);
        const float optical_y = static_cast<float>((static_cast<double>(v) - cy) * depth_m / fy);
        const Vec3f point = transform.apply(optical_x, optical_y, depth_m);

        if (!inside_crop(point)) {
          continue;
        }
        if (remove_self_ && inside_self(point)) {
          continue;
        }
        if (ground_filter_enabled_ && point.z >= ground_z_min_ && point.z <= ground_z_max_) {
          continue;
        }

        if (voxel_size_ > 0.0) {
          const int64_t key = voxel_key(point, nx, ny);
          if (!occupied_voxels.insert(key).second) {
            continue;
          }
        }
        points.push_back(point);
      }
    }

    if (points.empty()) {
      ++empty_dropped_frames_;
      return;
    }

    auto cloud = make_xyz_cloud(image->header, output_frame_, points);
    cloud_pub_->publish(cloud);
    ++published_frames_;

    const auto callback_end = steady_clock::now();
    if (last_output_time_.time_since_epoch().count() != 0) {
      const double dt = std::chrono::duration<double>(callback_end - last_output_time_).count();
      if (dt > 1.0e-6) {
        output_hz_ = ema(output_hz_, 1.0 / dt);
      }
    }
    last_output_time_ = callback_end;

    const double process_ms = duration_ms(callback_start, callback_end);
    const double output_age_ms = image_age_ms(image->header.stamp);
    const bool intrinsics_scaled = camera_info->width != image->width ||
      camera_info->height != image->height;

    if (last_stats_time_.time_since_epoch().count() == 0 ||
      std::chrono::duration<double>(callback_end - last_stats_time_).count() >= stats_period_sec_)
    {
      last_stats_time_ = callback_end;
      const uint64_t known_dropped = rate_dropped_frames_ + tf_dropped_frames_ +
        camera_info_dropped_frames_ + encoding_dropped_frames_ +
        invalid_image_dropped_frames_ + empty_dropped_frames_;

      std::ostringstream json;
      json.setf(std::ios::fixed);
      json.precision(2);
      json << "{"
           << "\"image_width\":" << image->width << ","
           << "\"image_height\":" << image->height << ","
           << "\"camera_info_width\":" << camera_info->width << ","
           << "\"camera_info_height\":" << camera_info->height << ","
           << "\"input_pixels\":" << static_cast<uint64_t>(image->width) * image->height << ","
           << "\"sampled_pixels\":" << sampled_pixels << ","
           << "\"valid_depth_pixels\":" << valid_depth_pixels << ","
           << "\"output_points\":" << points.size() << ","
           << "\"input_age_ms\":" << arrival_age_ms << ","
           << "\"output_age_ms\":" << output_age_ms << ","
           << "\"process_ms\":" << process_ms << ","
           << "\"tf_wait_ms\":" << tf_wait_ms << ","
           << "\"input_hz\":" << input_hz_ << ","
           << "\"output_hz\":" << output_hz_ << ","
           << "\"received_frames\":" << received_frames_ << ","
           << "\"published_frames\":" << published_frames_ << ","
           << "\"known_dropped_frames\":" << known_dropped << ","
           << "\"rate_dropped_frames\":" << rate_dropped_frames_ << ","
           << "\"tf_dropped_frames\":" << tf_dropped_frames_ << ","
           << "\"camera_info_dropped_frames\":" << camera_info_dropped_frames_ << ","
           << "\"encoding_dropped_frames\":" << encoding_dropped_frames_ << ","
           << "\"invalid_image_dropped_frames\":" << invalid_image_dropped_frames_ << ","
           << "\"empty_dropped_frames\":" << empty_dropped_frames_ << ","
           << "\"pixel_stride\":" << pixel_stride_ << ","
           << "\"voxel_size_m\":" << voxel_size_ << ","
           << "\"intrinsics_scaled\":" << (intrinsics_scaled ? "true" : "false") << ","
           << "\"encoding\":\"" << image->encoding << "\","
           << "\"source_frame\":\"" << image->header.frame_id << "\","
           << "\"output_frame\":\"" << output_frame_ << "\""
           << "}";

      std_msgs::msg::String stats;
      stats.data = json.str();
      stats_pub_->publish(stats);

      RCLCPP_INFO(
        get_logger(),
        "STEP10V2 %ux%u stride=%d sampled=%zu valid=%zu -> %zu points, "
        "process=%.1fms age=%.1fms output=%.1fHz%s",
        image->width, image->height, pixel_stride_, sampled_pixels, valid_depth_pixels,
        points.size(), process_ms, output_age_ms, output_hz_,
        intrinsics_scaled ? " [scaled intrinsics]" : "");
    }
  }

  void publish_markers()
  {
    if (!publish_markers_) {
      return;
    }
    const auto stamp = now();
    MarkerArray array;
    array.markers.push_back(cube_marker(
      0, "step10v2_crop", output_frame_, stamp,
      {x_min_, y_min_, z_min_}, {x_max_, y_max_, z_max_},
      {0.10F, 0.75F, 1.00F, 0.055F}));
    if (remove_self_) {
      array.markers.push_back(cube_marker(
        1, "step10v2_self", output_frame_, stamp,
        {self_x_min_, self_y_min_, self_z_min_},
        {self_x_max_, self_y_max_, self_z_max_},
        {1.00F, 0.20F, 0.15F, 0.15F}));
    }
    if (ground_filter_enabled_) {
      array.markers.push_back(cube_marker(
        2, "step10v2_ground", output_frame_, stamp,
        {x_min_, y_min_, ground_z_min_}, {x_max_, y_max_, ground_z_max_},
        {1.00F, 0.75F, 0.10F, 0.12F}));
    }
    marker_pub_->publish(array);
  }

  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string output_topic_;
  std::string stats_topic_;
  std::string marker_topic_;
  std::string output_frame_;

  double max_rate_hz_{30.0};
  int pixel_stride_{2};
  double depth_unit_scale_{0.001};
  double min_range_{0.20};
  double max_range_{4.0};
  double voxel_size_{0.03};
  double transform_timeout_sec_{0.015};
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
  bool publish_markers_{true};
  double stats_period_sec_{1.0};

  mutable std::mutex camera_info_mutex_;
  std::optional<CameraInfo> latest_camera_info_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stats_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr marker_timer_;

  steady_clock::time_point last_input_time_{};
  steady_clock::time_point last_output_time_{};
  steady_clock::time_point last_process_time_{};
  steady_clock::time_point last_stats_time_{};
  double input_hz_{0.0};
  double output_hz_{0.0};

  uint64_t received_frames_{0};
  uint64_t published_frames_{0};
  uint64_t rate_dropped_frames_{0};
  uint64_t tf_dropped_frames_{0};
  uint64_t camera_info_dropped_frames_{0};
  uint64_t encoding_dropped_frames_{0};
  uint64_t invalid_image_dropped_frames_{0};
  uint64_t empty_dropped_frames_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthImageToLocalCloudNode>());
  rclcpp::shutdown();
  return 0;
}
