#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;

uint32_t crc32_bytes(uint32_t crc, const uint8_t * data, size_t size)
{
  for (size_t index = 0; index < size; ++index) {
    crc ^= data[index];
    for (int bit = 0; bit < 8; ++bit) {
      const uint32_t mask = 0U - (crc & 1U);
      crc = (crc >> 1U) ^ (0xEDB88320U & mask);
    }
  }
  return crc;
}

template<typename ValueT>
uint32_t crc32_value(uint32_t crc, const ValueT & value)
{
  return crc32_bytes(
    crc, reinterpret_cast<const uint8_t *>(&value), sizeof(ValueT));
}

uint32_t occupancy_grid_crc32(const nav_msgs::msg::OccupancyGrid & map)
{
  uint32_t crc = 0xFFFFFFFFU;
  crc = crc32_value(crc, map.info.width);
  crc = crc32_value(crc, map.info.height);
  crc = crc32_value(crc, map.info.resolution);
  const auto & pose = map.info.origin;
  crc = crc32_value(crc, pose.position.x);
  crc = crc32_value(crc, pose.position.y);
  crc = crc32_value(crc, pose.position.z);
  crc = crc32_value(crc, pose.orientation.x);
  crc = crc32_value(crc, pose.orientation.y);
  crc = crc32_value(crc, pose.orientation.z);
  crc = crc32_value(crc, pose.orientation.w);
  if (!map.data.empty()) {
    crc = crc32_bytes(
      crc, reinterpret_cast<const uint8_t *>(map.data.data()), map.data.size());
  }
  return crc ^ 0xFFFFFFFFU;
}

double normalize_angle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

tf2::Quaternion valid_quaternion(
  double x, double y, double z, double w)
{
  tf2::Quaternion quaternion(x, y, z, w);
  if (quaternion.length2() < 1.0e-12) {
    quaternion.setValue(0.0, 0.0, 0.0, 1.0);
  } else {
    quaternion.normalize();
  }
  return quaternion;
}
}  // namespace

class MutableNavigationMapNode final : public rclcpp::Node
{
public:
  MutableNavigationMapNode()
  : Node("mutable_navigation_map"),
    tf_buffer_(std::make_unique<tf2_ros::Buffer>(get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    reference_topic_ = declare_parameter<std::string>(
      "reference_map_topic", "/map");
    output_topic_ = declare_parameter<std::string>(
      "output_map_topic", "/navigation_live_map");
    update_topic_ = declare_parameter<std::string>(
      "update_topic", "/navigation_live_map_updates");
    scan_topic_ = declare_parameter<std::string>(
      "scan_topic", "/scan_timed_v2_filtered");
    ready_topic_ = declare_parameter<std::string>(
      "localization_ready_topic", "/localization_ready");
    correction_hold_topic_ = declare_parameter<std::string>(
      "slam_correction_hold_topic", "/slam_correction_hold");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");

    occupied_threshold_ = std::clamp(
      static_cast<int>(declare_parameter<int64_t>("occupied_threshold", 65)),
      1, 100);
    mark_confirmations_ = std::clamp(
      static_cast<int>(declare_parameter<int64_t>("mark_confirmations", 3)),
      1, 255);
    clear_confirmations_ = std::clamp(
      static_cast<int>(declare_parameter<int64_t>("clear_confirmations", 20)),
      1, 255);
    max_evidence_rate_hz_ = std::clamp(
      declare_parameter<double>("max_evidence_rate_hz", 5.0), 0.5, 30.0);
    publish_rate_hz_ = std::clamp(
      declare_parameter<double>("update_publish_rate_hz", 2.0), 0.2, 10.0);
    full_publish_period_sec_ = std::max(
      5.0, declare_parameter<double>("full_publish_period_sec", 30.0));
    max_ray_range_ = std::max(
      0.5, declare_parameter<double>("max_ray_range", 12.0));
    endpoint_clearance_m_ = std::clamp(
      declare_parameter<double>("endpoint_clearance_m", 0.12), 0.05, 0.30);
    tf_timeout_sec_ = std::clamp(
      declare_parameter<double>("tf_timeout_sec", 0.05), 0.01, 0.50);
    pose_jump_translation_m_ = std::max(
      0.10, declare_parameter<double>("pose_jump_translation_m", 0.35));
    pose_jump_yaw_rad_ = std::max(
      5.0, declare_parameter<double>("pose_jump_yaw_deg", 20.0)) * kPi / 180.0;
    freeze_after_jump_sec_ = std::max(
      0.5, declare_parameter<double>("freeze_after_pose_jump_sec", 2.0));
    restore_on_pose_jump_ = declare_parameter<bool>(
      "restore_reference_on_pose_jump", true);
    clear_with_infinite_ranges_ = declare_parameter<bool>(
      "clear_with_infinite_ranges", true);

    const auto map_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    const auto update_qos =
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      output_topic_, map_qos);
    update_pub_ = create_publisher<map_msgs::msg::OccupancyGridUpdate>(
      update_topic_, update_qos);
    reference_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      reference_topic_, map_qos,
      std::bind(
        &MutableNavigationMapNode::on_reference_map, this,
        std::placeholders::_1));
    ready_sub_ = create_subscription<std_msgs::msg::Bool>(
      ready_topic_, map_qos,
      std::bind(
        &MutableNavigationMapNode::on_localization_ready, this,
        std::placeholders::_1));
    correction_hold_sub_ = create_subscription<std_msgs::msg::Bool>(
      correction_hold_topic_, map_qos,
      std::bind(
        &MutableNavigationMapNode::on_slam_correction_hold, this,
        std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS().keep_last(5),
      std::bind(
        &MutableNavigationMapNode::on_scan, this,
        std::placeholders::_1));

    update_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int64_t>(
        std::llround(1000.0 / publish_rate_hz_))),
      std::bind(&MutableNavigationMapNode::publish_pending_update, this));
    full_map_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int64_t>(
        std::llround(1000.0 * full_publish_period_sec_))),
      std::bind(&MutableNavigationMapNode::publish_full_map, this));
    status_timer_ = create_wall_timer(
      std::chrono::seconds(10),
      std::bind(&MutableNavigationMapNode::report_status, this));

    RCLCPP_INFO(
      get_logger(),
      "Mutable navigation map: reference=%s, output=%s + %s, scan=%s, "
      "mark=%d scans, clear=%d scans at <=%.1fHz",
      reference_topic_.c_str(), output_topic_.c_str(), update_topic_.c_str(),
      scan_topic_.c_str(), mark_confirmations_, clear_confirmations_,
      max_evidence_rate_hz_);
  }

private:
  void on_reference_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    const size_t expected =
      static_cast<size_t>(msg->info.width) * msg->info.height;
    if (msg->info.resolution <= 0.0F || msg->info.width == 0U ||
      msg->info.height == 0U || msg->data.size() != expected)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Rejected invalid reference map: %ux%u resolution=%.6f data=%zu",
        msg->info.width, msg->info.height, msg->info.resolution,
        msg->data.size());
      return;
    }

    const uint32_t incoming_crc = occupancy_grid_crc32(*msg);
    if (reference_locked_) {
      if (incoming_crc != reference_crc32_) {
        RCLCPP_ERROR(
          get_logger(),
          "NAV_MAP_REFERENCE_MUTATION_REJECTED locked=0x%08x incoming=0x%08x",
          reference_crc32_, incoming_crc);
      }
      return;
    }
    reference_locked_ = true;
    reference_crc32_ = incoming_crc;

    reference_map_ = *msg;
    reference_map_.header.frame_id = map_frame_;
    current_map_ = reference_map_;
    map_loaded_ = true;
    const auto & pose = current_map_.info.origin;
    const tf2::Quaternion orientation = valid_quaternion(
      pose.orientation.x, pose.orientation.y, pose.orientation.z,
      pose.orientation.w);
    const tf2::Transform map_from_grid(
      orientation,
      tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
    grid_from_map_ = map_from_grid.inverse();

    mark_evidence_.assign(expected, 0U);
    clear_evidence_.assign(expected, 0U);
    free_generation_.assign(expected, 0U);
    occupied_generation_.assign(expected, 0U);
    free_cells_.clear();
    occupied_cells_.clear();
    scan_generation_ = 0U;
    reset_evidence();
    reset_pending_bounds();
    publish_full_map();
    RCLCPP_INFO(
      get_logger(),
      "NAV_MAP_REFERENCE loaded=%ux%u resolution=%.3fm cells=%zu "
      "crc32=0x%08x; source remains immutable",
      current_map_.info.width, current_map_.info.height,
      current_map_.info.resolution, expected, reference_crc32_);
  }

  void on_localization_ready(const std_msgs::msg::Bool::SharedPtr msg)
  {
    const bool was_ready = localization_ready_;
    localization_ready_ = msg->data;
    if (localization_ready_ == was_ready) {
      return;
    }

    reset_evidence();
    have_last_pose_ = false;
    last_evidence_stamp_ns_ = 0;
    if (!localization_ready_) {
      if (was_ready && map_loaded_) {
        restore_reference_map("localization paused or invalidated");
      }
      RCLCPP_WARN(
        get_logger(),
        "NAV_MAP_FROZEN localization_ready=false; evidence updates stopped");
      return;
    }

    freeze_until_ = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(0.5));
    RCLCPP_INFO(
      get_logger(),
      "NAV_MAP_ACTIVE verified localization received; live map evidence enabled");
  }

  void on_slam_correction_hold(const std_msgs::msg::Bool::SharedPtr msg)
  {
    const bool was_active = correction_hold_active_;
    correction_hold_active_ = msg->data;
    if (correction_hold_active_ == was_active) {
      return;
    }
    if (!correction_hold_active_) {
      RCLCPP_INFO(
        get_logger(),
        "NAV_MAP_LOOP_RELEASE correction hold ended; evidence resumes after freeze");
      return;
    }

    ++loop_correction_resets_;
    reset_evidence();
    have_last_pose_ = false;
    last_evidence_stamp_ns_ = 0;
    if (restore_on_pose_jump_) {
      restore_reference_map("Cartographer loop correction");
    }
    freeze_until_ = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(freeze_after_jump_sec_));
    RCLCPP_WARN(
      get_logger(),
      "NAV_MAP_LOOP_CORRECTION evidence reset, live edits restored from the "
      "immutable reference, updates frozen for %.1fs",
      freeze_after_jump_sec_);
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    ++scans_received_;
    if (!map_loaded_ || !localization_ready_) {
      ++scans_not_ready_;
      return;
    }
    if (correction_hold_active_) {
      ++scans_frozen_;
      return;
    }
    if (scan->header.frame_id.empty() || scan->ranges.empty()) {
      ++invalid_scans_;
      return;
    }

    const rclcpp::Time stamp(scan->header.stamp);
    const int64_t stamp_ns = stamp.nanoseconds();
    if (stamp_ns <= 0) {
      ++invalid_scans_;
      return;
    }
    const int64_t min_interval_ns = static_cast<int64_t>(
      1.0e9 / max_evidence_rate_hz_);
    if (last_evidence_stamp_ns_ > 0 &&
      stamp_ns - last_evidence_stamp_ns_ < min_interval_ns)
    {
      ++scans_throttled_;
      return;
    }
    if (last_evidence_stamp_ns_ > 0 && stamp_ns <= last_evidence_stamp_ns_) {
      ++invalid_scans_;
      reset_evidence();
      last_evidence_stamp_ns_ = 0;
      return;
    }
    last_evidence_stamp_ns_ = stamp_ns;

    geometry_msgs::msg::TransformStamped transform_msg;
    try {
      transform_msg = tf_buffer_->lookupTransform(
        map_frame_, scan->header.frame_id, stamp,
        rclcpp::Duration::from_seconds(tf_timeout_sec_));
    } catch (const tf2::TransformException & error) {
      ++tf_rejects_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "NAV_MAP_TF_REJECT %s <- %s at scan stamp: %s",
        map_frame_.c_str(), scan->header.frame_id.c_str(), error.what());
      return;
    }

    const auto & translation = transform_msg.transform.translation;
    const auto & rotation = transform_msg.transform.rotation;
    const tf2::Quaternion quaternion = valid_quaternion(
      rotation.x, rotation.y, rotation.z, rotation.w);
    const tf2::Transform map_from_sensor(
      quaternion,
      tf2::Vector3(translation.x, translation.y, translation.z));
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    if (have_last_pose_) {
      const double dx = translation.x - last_pose_x_;
      const double dy = translation.y - last_pose_y_;
      const double distance = std::hypot(dx, dy);
      const double yaw_delta = std::abs(normalize_angle(yaw - last_pose_yaw_));
      if (distance > pose_jump_translation_m_ || yaw_delta > pose_jump_yaw_rad_) {
        ++pose_jump_resets_;
        reset_evidence();
        if (restore_on_pose_jump_) {
          restore_reference_map("map-frame pose jump");
        }
        freeze_until_ = std::chrono::steady_clock::now() +
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(freeze_after_jump_sec_));
        RCLCPP_ERROR(
          get_logger(),
          "NAV_MAP_POSE_JUMP translation=%.3fm yaw=%.1fdeg; evidence reset "
          "and updates frozen for %.1fs",
          distance, yaw_delta * 180.0 / kPi, freeze_after_jump_sec_);
        update_last_pose(translation.x, translation.y, yaw);
        return;
      }
    }
    update_last_pose(translation.x, translation.y, yaw);

    if (std::chrono::steady_clock::now() < freeze_until_) {
      ++scans_frozen_;
      return;
    }

    int origin_x = 0;
    int origin_y = 0;
    if (!world_to_map(translation.x, translation.y, origin_x, origin_y)) {
      ++origin_outside_map_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "NAV_MAP_REJECT laser origin is outside selected map (%.2f, %.2f)",
        translation.x, translation.y);
      return;
    }

    const auto started = std::chrono::steady_clock::now();
    next_scan_generation();
    free_cells_.clear();
    occupied_cells_.clear();

    double angle = static_cast<double>(scan->angle_min);
    const double range_min = std::max(0.0, static_cast<double>(scan->range_min));
    const double sensor_range_max =
      std::isfinite(scan->range_max) && scan->range_max > 0.0F ?
      static_cast<double>(scan->range_max) : max_ray_range_;
    const double ray_limit = std::min(sensor_range_max, max_ray_range_);
    for (const float raw_range : scan->ranges) {
      const bool finite = std::isfinite(raw_range);
      const bool infinite = std::isinf(raw_range) && raw_range > 0.0F;
      if ((!finite && !(infinite && clear_with_infinite_ranges_)) ||
        (finite && static_cast<double>(raw_range) < range_min))
      {
        angle += static_cast<double>(scan->angle_increment);
        continue;
      }

      const double measured_range = finite ?
        static_cast<double>(raw_range) : ray_limit;
      const double ray_range = std::min(measured_range, ray_limit);
      if (ray_range < range_min || ray_range <= 0.0) {
        angle += static_cast<double>(scan->angle_increment);
        continue;
      }
      const bool has_hit = finite && measured_range <= max_ray_range_ &&
        measured_range < sensor_range_max - 0.02;
      const double clear_range = has_hit ?
        std::max(range_min, ray_range - endpoint_clearance_m_) : ray_range;

      const tf2::Vector3 clear_sensor(
        clear_range * std::cos(angle), clear_range * std::sin(angle), 0.0);
      const tf2::Vector3 clear_map = map_from_sensor * clear_sensor;
      int clear_x = 0;
      int clear_y = 0;
      // world_to_map still returns the unbounded grid coordinates when the
      // endpoint is outside the selected PGM. Walk that ray anyway so cells
      // inside the map are not skipped near map boundaries.
      world_to_map(clear_map.x(), clear_map.y(), clear_x, clear_y);
      collect_ray_cells(origin_x, origin_y, clear_x, clear_y);

      if (has_hit) {
        const tf2::Vector3 hit_sensor(
          measured_range * std::cos(angle),
          measured_range * std::sin(angle), 0.0);
        const tf2::Vector3 hit_map = map_from_sensor * hit_sensor;
        int hit_x = 0;
        int hit_y = 0;
        if (world_to_map(hit_map.x(), hit_map.y(), hit_x, hit_y)) {
          add_occupied_cell(cell_index(hit_x, hit_y));
        }
      }
      angle += static_cast<double>(scan->angle_increment);
    }

    apply_evidence();
    ++scans_processed_;
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    processing_ms_total_ += elapsed_ms;
    processing_ms_max_ = std::max(processing_ms_max_, elapsed_ms);
  }

  void update_last_pose(double x, double y, double yaw)
  {
    last_pose_x_ = x;
    last_pose_y_ = y;
    last_pose_yaw_ = yaw;
    have_last_pose_ = true;
  }

  void next_scan_generation()
  {
    ++scan_generation_;
    if (scan_generation_ == 0U) {
      std::fill(free_generation_.begin(), free_generation_.end(), 0U);
      std::fill(occupied_generation_.begin(), occupied_generation_.end(), 0U);
      scan_generation_ = 1U;
    }
  }

  bool world_to_map(double world_x, double world_y, int & map_x, int & map_y) const
  {
    if (!map_loaded_) {
      return false;
    }
    const tf2::Vector3 grid_point =
      grid_from_map_ * tf2::Vector3(world_x, world_y, 0.0);
    const double resolution = static_cast<double>(current_map_.info.resolution);
    map_x = static_cast<int>(std::floor(grid_point.x() / resolution));
    map_y = static_cast<int>(std::floor(grid_point.y() / resolution));
    return map_x >= 0 && map_y >= 0 &&
      map_x < static_cast<int>(current_map_.info.width) &&
      map_y < static_cast<int>(current_map_.info.height);
  }

  size_t cell_index(int x, int y) const
  {
    return static_cast<size_t>(y) * current_map_.info.width +
      static_cast<size_t>(x);
  }

  void add_free_cell(size_t index)
  {
    if (free_generation_[index] == scan_generation_) {
      return;
    }
    free_generation_[index] = scan_generation_;
    free_cells_.push_back(index);
  }

  void add_occupied_cell(size_t index)
  {
    if (occupied_generation_[index] == scan_generation_) {
      return;
    }
    occupied_generation_[index] = scan_generation_;
    occupied_cells_.push_back(index);
  }

  void collect_ray_cells(int x0, int y0, int x1, int y1)
  {
    int dx = std::abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int error = dx + dy;
    int x = x0;
    int y = y0;
    while (true) {
      if (x >= 0 && y >= 0 &&
        x < static_cast<int>(current_map_.info.width) &&
        y < static_cast<int>(current_map_.info.height))
      {
        add_free_cell(cell_index(x, y));
      }
      if (x == x1 && y == y1) {
        break;
      }
      const int twice_error = 2 * error;
      if (twice_error >= dy) {
        error += dy;
        x += sx;
      }
      if (twice_error <= dx) {
        error += dx;
        y += sy;
      }
    }
  }

  void apply_evidence()
  {
    for (const size_t index : free_cells_) {
      if (occupied_generation_[index] == scan_generation_) {
        continue;
      }
      mark_evidence_[index] = 0U;
      const int value = static_cast<int>(current_map_.data[index]);
      if (value < occupied_threshold_) {
        clear_evidence_[index] = 0U;
        continue;
      }
      uint8_t & evidence = clear_evidence_[index];
      evidence = static_cast<uint8_t>(std::min(
        255, static_cast<int>(evidence) + 1));
      if (static_cast<int>(evidence) >= clear_confirmations_) {
        current_map_.data[index] = 0;
        evidence = 0U;
        ++cells_cleared_;
        record_changed(index);
      }
    }

    for (const size_t index : occupied_cells_) {
      clear_evidence_[index] = 0U;
      const int value = static_cast<int>(current_map_.data[index]);
      if (value >= occupied_threshold_) {
        mark_evidence_[index] = 0U;
        continue;
      }
      uint8_t & evidence = mark_evidence_[index];
      evidence = static_cast<uint8_t>(std::min(
        255, static_cast<int>(evidence) + 1));
      if (static_cast<int>(evidence) >= mark_confirmations_) {
        current_map_.data[index] = 100;
        evidence = 0U;
        ++cells_marked_;
        record_changed(index);
      }
    }
  }

  void record_changed(size_t index)
  {
    const int width = static_cast<int>(current_map_.info.width);
    const int x = static_cast<int>(index % current_map_.info.width);
    const int y = static_cast<int>(index / current_map_.info.width);
    if (!pending_update_) {
      pending_min_x_ = pending_max_x_ = x;
      pending_min_y_ = pending_max_y_ = y;
      pending_update_ = true;
      return;
    }
    pending_min_x_ = std::clamp(std::min(pending_min_x_, x), 0, width - 1);
    pending_max_x_ = std::clamp(std::max(pending_max_x_, x), 0, width - 1);
    pending_min_y_ = std::min(pending_min_y_, y);
    pending_max_y_ = std::max(pending_max_y_, y);
  }

  void publish_pending_update()
  {
    if (!map_loaded_ || !pending_update_) {
      return;
    }
    map_msgs::msg::OccupancyGridUpdate update;
    update.header.stamp = now();
    update.header.frame_id = map_frame_;
    update.x = pending_min_x_;
    update.y = pending_min_y_;
    update.width = static_cast<uint32_t>(pending_max_x_ - pending_min_x_ + 1);
    update.height = static_cast<uint32_t>(pending_max_y_ - pending_min_y_ + 1);
    update.data.reserve(static_cast<size_t>(update.width) * update.height);
    for (int y = pending_min_y_; y <= pending_max_y_; ++y) {
      const size_t begin = cell_index(pending_min_x_, y);
      const size_t end = begin + update.width;
      update.data.insert(
        update.data.end(), current_map_.data.begin() +
        static_cast<std::vector<int8_t>::difference_type>(begin),
        current_map_.data.begin() +
        static_cast<std::vector<int8_t>::difference_type>(end));
    }
    update_pub_->publish(update);
    ++updates_published_;
    changed_area_cells_total_ +=
      static_cast<uint64_t>(update.width) * update.height;
    reset_pending_bounds();
  }

  void publish_full_map()
  {
    if (!map_loaded_) {
      return;
    }
    current_map_.header.stamp = now();
    current_map_.header.frame_id = map_frame_;
    map_pub_->publish(current_map_);
    ++full_maps_published_;
    reset_pending_bounds();
  }

  void restore_reference_map(const char * reason)
  {
    if (!map_loaded_ || current_map_.data == reference_map_.data) {
      reset_evidence();
      reset_pending_bounds();
      return;
    }
    current_map_ = reference_map_;
    reset_evidence();
    reset_pending_bounds();
    publish_full_map();
    ++reference_restores_;
    RCLCPP_WARN(
      get_logger(), "NAV_MAP_RESTORE reference map restored: %s", reason);
  }

  void reset_evidence()
  {
    std::fill(mark_evidence_.begin(), mark_evidence_.end(), 0U);
    std::fill(clear_evidence_.begin(), clear_evidence_.end(), 0U);
  }

  void reset_pending_bounds()
  {
    pending_update_ = false;
    pending_min_x_ = pending_min_y_ = 0;
    pending_max_x_ = pending_max_y_ = 0;
  }

  void report_status()
  {
    const double average_ms = scans_processed_ > 0U ?
      processing_ms_total_ / static_cast<double>(scans_processed_) : 0.0;
    RCLCPP_INFO(
      get_logger(),
      "NAV_MAP_STATUS loaded=%s ready=%s scans=%llu processed=%llu "
      "not_ready=%llu throttled=%llu invalid=%llu tf_reject=%llu "
      "origin_outside=%llu frozen=%llu pose_jump=%llu loop_reset=%llu "
      "marked=%llu cleared=%llu updates=%llu update_cells=%llu "
      "full=%llu restores=%llu process_avg=%.2fms process_max=%.2fms",
      map_loaded_ ? "true" : "false",
      localization_ready_ ? "true" : "false",
      static_cast<unsigned long long>(scans_received_),
      static_cast<unsigned long long>(scans_processed_),
      static_cast<unsigned long long>(scans_not_ready_),
      static_cast<unsigned long long>(scans_throttled_),
      static_cast<unsigned long long>(invalid_scans_),
      static_cast<unsigned long long>(tf_rejects_),
      static_cast<unsigned long long>(origin_outside_map_),
      static_cast<unsigned long long>(scans_frozen_),
      static_cast<unsigned long long>(pose_jump_resets_),
      static_cast<unsigned long long>(loop_correction_resets_),
      static_cast<unsigned long long>(cells_marked_),
      static_cast<unsigned long long>(cells_cleared_),
      static_cast<unsigned long long>(updates_published_),
      static_cast<unsigned long long>(changed_area_cells_total_),
      static_cast<unsigned long long>(full_maps_published_),
      static_cast<unsigned long long>(reference_restores_), average_ms,
      processing_ms_max_);
  }

  std::string reference_topic_;
  std::string output_topic_;
  std::string update_topic_;
  std::string scan_topic_;
  std::string ready_topic_;
  std::string correction_hold_topic_;
  std::string map_frame_;

  bool reference_locked_{false};
  uint32_t reference_crc32_{0U};
  int occupied_threshold_{65};
  int mark_confirmations_{3};
  int clear_confirmations_{20};
  double max_evidence_rate_hz_{5.0};
  double publish_rate_hz_{2.0};
  double full_publish_period_sec_{30.0};
  double max_ray_range_{12.0};
  double endpoint_clearance_m_{0.12};
  double tf_timeout_sec_{0.05};
  double pose_jump_translation_m_{0.35};
  double pose_jump_yaw_rad_{20.0 * kPi / 180.0};
  double freeze_after_jump_sec_{2.0};
  bool restore_on_pose_jump_{true};
  bool clear_with_infinite_ranges_{true};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<map_msgs::msg::OccupancyGridUpdate>::SharedPtr update_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr reference_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr correction_hold_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::TimerBase::SharedPtr full_map_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  nav_msgs::msg::OccupancyGrid reference_map_;
  nav_msgs::msg::OccupancyGrid current_map_;
  tf2::Transform grid_from_map_;
  std::vector<uint8_t> mark_evidence_;
  std::vector<uint8_t> clear_evidence_;
  std::vector<uint32_t> free_generation_;
  std::vector<uint32_t> occupied_generation_;
  std::vector<size_t> free_cells_;
  std::vector<size_t> occupied_cells_;
  uint32_t scan_generation_{0U};
  bool map_loaded_{false};
  bool localization_ready_{false};
  bool correction_hold_active_{false};
  bool have_last_pose_{false};
  double last_pose_x_{0.0};
  double last_pose_y_{0.0};
  double last_pose_yaw_{0.0};
  int64_t last_evidence_stamp_ns_{0};
  std::chrono::steady_clock::time_point freeze_until_{};

  bool pending_update_{false};
  int pending_min_x_{0};
  int pending_min_y_{0};
  int pending_max_x_{0};
  int pending_max_y_{0};

  uint64_t scans_received_{0U};
  uint64_t scans_processed_{0U};
  uint64_t scans_not_ready_{0U};
  uint64_t scans_throttled_{0U};
  uint64_t scans_frozen_{0U};
  uint64_t invalid_scans_{0U};
  uint64_t tf_rejects_{0U};
  uint64_t origin_outside_map_{0U};
  uint64_t pose_jump_resets_{0U};
  uint64_t loop_correction_resets_{0U};
  uint64_t cells_marked_{0U};
  uint64_t cells_cleared_{0U};
  uint64_t updates_published_{0U};
  uint64_t full_maps_published_{0U};
  uint64_t reference_restores_{0U};
  uint64_t changed_area_cells_total_{0U};
  double processing_ms_total_{0.0};
  double processing_ms_max_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MutableNavigationMapNode>());
  rclcpp::shutdown();
  return 0;
}
