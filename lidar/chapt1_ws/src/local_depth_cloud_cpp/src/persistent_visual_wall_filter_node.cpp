#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace
{
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

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

int64_t column_key(int32_t ix, int32_t iy)
{
  const uint64_t packed =
    (static_cast<uint64_t>(static_cast<uint32_t>(ix)) << 32) |
    static_cast<uint32_t>(iy);
  return static_cast<int64_t>(packed);
}

struct ColumnStats
{
  float min_z{std::numeric_limits<float>::infinity()};
  float max_z{-std::numeric_limits<float>::infinity()};
  int count{0};
  int32_t ix{0};
  int32_t iy{0};
};
}  // namespace

class PersistentVisualWallFilter : public rclcpp::Node
{
public:
  PersistentVisualWallFilter()
  : Node("persistent_visual_wall_filter")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/rtabmap_3d/octomap_occupied_space");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/rtabmap_3d/navigation_walls");
    column_size_ = std::max(0.02, declare_parameter<double>("column_size", 0.05));
    neighborhood_cells_ = std::max(
      0, static_cast<int>(declare_parameter<int64_t>("neighborhood_cells", 1)));
    min_z_ = declare_parameter<double>("min_z", 0.08);
    max_z_ = declare_parameter<double>("max_z", 1.40);
    min_vertical_span_ = std::max(
      0.05, declare_parameter<double>("min_vertical_span", 0.35));
    min_column_points_ = std::max(
      2, static_cast<int>(declare_parameter<int64_t>("min_column_points", 5)));
    publish_all_column_points_ = declare_parameter<bool>(
      "publish_all_column_points", true);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    publisher_ = create_publisher<PointCloud2>(output_topic_, qos);
    subscription_ = create_subscription<PointCloud2>(
      input_topic_, qos,
      std::bind(
        &PersistentVisualWallFilter::on_cloud, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Persistent visual wall filter: %s -> %s, column=%.3fm, neighborhood=%d, "
      "z=[%.2f,%.2f], vertical_span>=%.2fm, points>=%d",
      input_topic_.c_str(), output_topic_.c_str(), column_size_,
      neighborhood_cells_, min_z_, max_z_, min_vertical_span_, min_column_points_);
  }

private:
  struct PointRef
  {
    size_t offset;
    int64_t key;
  };

  void on_cloud(const PointCloud2::SharedPtr cloud)
  {
    if (cloud->is_bigendian) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Big-endian PointCloud2 is unsupported by the visual wall filter");
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
        "Visual wall cloud must contain FLOAT32 x/y/z fields");
      return;
    }
    const uint32_t required_size =
      std::max({x_offset, y_offset, z_offset}) + sizeof(float);
    if (cloud->point_step < required_size || cloud->row_step == 0U) {
      return;
    }

    std::unordered_map<int64_t, ColumnStats> columns;
    std::vector<PointRef> candidates;
    candidates.reserve(static_cast<size_t>(cloud->width) * cloud->height);

    for (uint32_t row = 0U; row < cloud->height; ++row) {
      const size_t row_base = static_cast<size_t>(row) * cloud->row_step;
      for (uint32_t col = 0U; col < cloud->width; ++col) {
        const size_t base =
          row_base + static_cast<size_t>(col) * cloud->point_step;
        if (base + required_size > cloud->data.size()) {
          break;
        }
        const uint8_t * point = cloud->data.data() + base;
        const float x = read_float32(point, x_offset);
        const float y = read_float32(point, y_offset);
        const float z = read_float32(point, z_offset);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
          z < min_z_ || z > max_z_)
        {
          continue;
        }

        const int32_t ix = static_cast<int32_t>(
          std::floor(static_cast<double>(x) / column_size_));
        const int32_t iy = static_cast<int32_t>(
          std::floor(static_cast<double>(y) / column_size_));
        const int64_t key = column_key(ix, iy);
        auto & stats = columns[key];
        stats.ix = ix;
        stats.iy = iy;
        stats.min_z = std::min(stats.min_z, z);
        stats.max_z = std::max(stats.max_z, z);
        ++stats.count;
        candidates.push_back({base, key});
      }
    }

    std::unordered_map<int64_t, bool> accepted;
    accepted.reserve(columns.size());
    for (const auto & entry : columns) {
      const int32_t ix = entry.second.ix;
      const int32_t iy = entry.second.iy;
      ColumnStats stats;
      for (int dx = -neighborhood_cells_; dx <= neighborhood_cells_; ++dx) {
        for (int dy = -neighborhood_cells_; dy <= neighborhood_cells_; ++dy) {
          const auto neighbor = columns.find(column_key(ix + dx, iy + dy));
          if (neighbor == columns.end()) {
            continue;
          }
          stats.min_z = std::min(stats.min_z, neighbor->second.min_z);
          stats.max_z = std::max(stats.max_z, neighbor->second.max_z);
          stats.count += neighbor->second.count;
        }
      }
      accepted.emplace(
        entry.first,
        stats.count >= min_column_points_ &&
        static_cast<double>(stats.max_z - stats.min_z) >= min_vertical_span_);
    }

    PointCloud2 output = *cloud;
    // The cloud contains the latest optimized map coordinates. Restamping at
    // publication avoids costmap drops when rebuilding a large RTAB grid took
    // longer than Nav2's TF cache tolerance.
    output.header.stamp = get_clock()->now();
    output.height = 1U;
    output.width = 0U;
    output.row_step = 0U;
    output.data.clear();
    output.data.reserve(candidates.size() * cloud->point_step);

    std::unordered_map<int64_t, bool> emitted;
    size_t accepted_columns = 0U;
    for (const auto & entry : accepted) {
      if (entry.second) {
        ++accepted_columns;
      }
    }
    if (!publish_all_column_points_) {
      emitted.reserve(accepted.size());
    }
    for (const auto & candidate : candidates) {
      const auto accepted_it = accepted.find(candidate.key);
      if (accepted_it == accepted.end() || !accepted_it->second) {
        continue;
      }
      if (!publish_all_column_points_ && emitted[candidate.key]) {
        continue;
      }
      const auto begin = cloud->data.begin() +
        static_cast<std::vector<uint8_t>::difference_type>(candidate.offset);
      output.data.insert(
        output.data.end(), begin,
        begin + static_cast<std::vector<uint8_t>::difference_type>(
          cloud->point_step));
      ++output.width;
      if (!publish_all_column_points_) {
        emitted[candidate.key] = true;
      }
    }
    output.row_step = output.width * output.point_step;
    output.is_dense = false;
    publisher_->publish(output);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "VISUAL_WALL_FILTER input=%zu candidate_columns=%zu accepted_columns=%zu output=%u",
      static_cast<size_t>(cloud->width) * cloud->height,
      columns.size(), accepted_columns, output.width);
  }

  std::string input_topic_;
  std::string output_topic_;
  double column_size_{0.05};
  int neighborhood_cells_{1};
  double min_z_{0.08};
  double max_z_{1.40};
  double min_vertical_span_{0.35};
  int min_column_points_{5};
  bool publish_all_column_points_{true};
  rclcpp::Subscription<PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PersistentVisualWallFilter>());
  rclcpp::shutdown();
  return 0;
}
