#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{
class MetricWindow
{
public:
  explicit MetricWindow(size_t capacity)
  : data_(std::max<size_t>(capacity, 1U), 0.0)
  {
  }

  void add(double value)
  {
    if (!std::isfinite(value) || value < 0.0) {
      return;
    }
    data_[next_] = value;
    next_ = (next_ + 1U) % data_.size();
    count_ = std::min(count_ + 1U, data_.size());
  }

  struct Snapshot
  {
    double avg{0.0};
    double p95{0.0};
    double max{0.0};
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
      const double value = data_[i];
      copy.push_back(value);
      sum += value;
      out.max = std::max(out.max, value);
    }
    out.avg = sum / static_cast<double>(count_);
    std::sort(copy.begin(), copy.end());
    const size_t index = static_cast<size_t>(
      std::ceil(0.95 * static_cast<double>(copy.size()))) - 1U;
    out.p95 = copy[std::min(index, copy.size() - 1U)];
    return out;
  }

private:
  std::vector<double> data_;
  size_t next_{0U};
  size_t count_{0U};
};

int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL +
    static_cast<int64_t>(stamp.nanosec);
}
}  // namespace

class RgbdTimestampMonitorNode final : public rclcpp::Node
{
public:
  RgbdTimestampMonitorNode()
  : Node("rgbd_timestamp_monitor")
  {
    color_topic_ = declare_parameter<std::string>(
      "color_topic", "/camera/color/image_raw");
    depth_topic_ = declare_parameter<std::string>(
      "depth_topic", "/camera/depth/image_raw");
    stats_topic_ = declare_parameter<std::string>(
      "stats_topic", "/rgbd_timestamp_sync/stats");
    max_pair_interval_ms_ = declare_parameter<double>("max_pair_interval_ms", 40.0);
    warn_p95_ms_ = declare_parameter<double>("warn_p95_ms", 25.0);
    const int window_size = std::max(30, static_cast<int>(declare_parameter<int>("window_size", 300)));
    pair_diff_window_ = std::make_unique<MetricWindow>(static_cast<size_t>(window_size));
    color_gap_window_ = std::make_unique<MetricWindow>(static_cast<size_t>(window_size));
    depth_gap_window_ = std::make_unique<MetricWindow>(static_cast<size_t>(window_size));

    const auto qos = rclcpp::SensorDataQoS().keep_last(5);
    color_sub_ = create_subscription<sensor_msgs::msg::Image>(
      color_topic_, qos,
      std::bind(&RgbdTimestampMonitorNode::color_callback, this, std::placeholders::_1));
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, qos,
      std::bind(&RgbdTimestampMonitorNode::depth_callback, this, std::placeholders::_1));
    stats_pub_ = create_publisher<std_msgs::msg::String>(stats_topic_, rclcpp::QoS(10));
    timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&RgbdTimestampMonitorNode::publish_stats, this));

    RCLCPP_INFO(
      get_logger(), "RGB-D timestamp monitor: %s + %s -> %s",
      color_topic_.c_str(), depth_topic_.c_str(), stats_topic_.c_str());
  }

private:
  void color_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    handle_stamp(true, stamp_ns(msg->header.stamp));
  }

  void depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    handle_stamp(false, stamp_ns(msg->header.stamp));
  }

  void handle_stamp(bool is_color, int64_t stamp)
  {
    if (stamp == 0) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    auto & queue = is_color ? color_queue_ : depth_queue_;
    int64_t & last_stamp = is_color ? last_color_stamp_ns_ : last_depth_stamp_ns_;
    auto & gap_window = is_color ? color_gap_window_ : depth_gap_window_;
    auto & received = is_color ? color_received_ : depth_received_;
    auto & duplicate = is_color ? color_duplicate_ : depth_duplicate_;
    auto & backward = is_color ? color_backward_ : depth_backward_;

    ++received;
    if (last_stamp != 0) {
      if (stamp == last_stamp) {
        ++duplicate;
      } else if (stamp < last_stamp) {
        ++backward;
      } else {
        gap_window->add(static_cast<double>(stamp - last_stamp) / 1.0e6);
      }
    }
    if (stamp > last_stamp) {
      last_stamp = stamp;
    }
    queue.push_back(stamp);
    while (queue.size() > 30U) {
      queue.pop_front();
      if (is_color) {
        ++color_unmatched_;
      } else {
        ++depth_unmatched_;
      }
    }
    pair_queues_locked();
  }

  void pair_queues_locked()
  {
    const int64_t max_pair_ns = static_cast<int64_t>(max_pair_interval_ms_ * 1.0e6);
    while (!color_queue_.empty() && !depth_queue_.empty()) {
      const int64_t color = color_queue_.front();
      const int64_t depth = depth_queue_.front();
      const int64_t diff = color - depth;
      if (std::llabs(diff) <= max_pair_ns) {
        pair_diff_window_->add(static_cast<double>(std::llabs(diff)) / 1.0e6);
        latest_signed_diff_ms_ = static_cast<double>(diff) / 1.0e6;
        color_queue_.pop_front();
        depth_queue_.pop_front();
        ++paired_frames_;
      } else if (color < depth) {
        color_queue_.pop_front();
        ++color_unmatched_;
      } else {
        depth_queue_.pop_front();
        ++depth_unmatched_;
      }
    }
  }

  void publish_stats()
  {
    MetricWindow::Snapshot pair;
    MetricWindow::Snapshot color_gap;
    MetricWindow::Snapshot depth_gap;
    double latest_signed_diff = 0.0;
    uint64_t color_received = 0U;
    uint64_t depth_received = 0U;
    uint64_t paired = 0U;
    uint64_t color_unmatched = 0U;
    uint64_t depth_unmatched = 0U;
    uint64_t color_duplicate = 0U;
    uint64_t depth_duplicate = 0U;
    uint64_t color_backward = 0U;
    uint64_t depth_backward = 0U;
    size_t color_queue_size = 0U;
    size_t depth_queue_size = 0U;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pair = pair_diff_window_->snapshot();
      color_gap = color_gap_window_->snapshot();
      depth_gap = depth_gap_window_->snapshot();
      latest_signed_diff = latest_signed_diff_ms_;
      color_received = color_received_;
      depth_received = depth_received_;
      paired = paired_frames_;
      color_unmatched = color_unmatched_;
      depth_unmatched = depth_unmatched_;
      color_duplicate = color_duplicate_;
      depth_duplicate = depth_duplicate_;
      color_backward = color_backward_;
      depth_backward = depth_backward_;
      color_queue_size = color_queue_.size();
      depth_queue_size = depth_queue_.size();
    }

    const double color_hz = color_gap.avg > 0.0 ? 1000.0 / color_gap.avg : 0.0;
    const double depth_hz = depth_gap.avg > 0.0 ? 1000.0 / depth_gap.avg : 0.0;
    std::ostringstream json;
    json.setf(std::ios::fixed);
    json.precision(3);
    json << "{"
         << "\"latest_signed_diff_ms\":" << latest_signed_diff << ","
         << "\"abs_diff_avg_ms\":" << pair.avg << ","
         << "\"abs_diff_p95_ms\":" << pair.p95 << ","
         << "\"abs_diff_max_ms\":" << pair.max << ","
         << "\"color_gap_avg_ms\":" << color_gap.avg << ","
         << "\"color_gap_p95_ms\":" << color_gap.p95 << ","
         << "\"color_gap_max_ms\":" << color_gap.max << ","
         << "\"depth_gap_avg_ms\":" << depth_gap.avg << ","
         << "\"depth_gap_p95_ms\":" << depth_gap.p95 << ","
         << "\"depth_gap_max_ms\":" << depth_gap.max << ","
         << "\"color_hz\":" << color_hz << ","
         << "\"depth_hz\":" << depth_hz << ","
         << "\"color_received\":" << color_received << ","
         << "\"depth_received\":" << depth_received << ","
         << "\"paired_frames\":" << paired << ","
         << "\"color_unmatched\":" << color_unmatched << ","
         << "\"depth_unmatched\":" << depth_unmatched << ","
         << "\"color_duplicate\":" << color_duplicate << ","
         << "\"depth_duplicate\":" << depth_duplicate << ","
         << "\"color_backward\":" << color_backward << ","
         << "\"depth_backward\":" << depth_backward << ","
         << "\"color_queue_size\":" << color_queue_size << ","
         << "\"depth_queue_size\":" << depth_queue_size
         << "}";

    std_msgs::msg::String stats;
    stats.data = json.str();
    stats_pub_->publish(stats);

    if (pair.count > 10U && pair.p95 > warn_p95_ms_) {
      RCLCPP_WARN(
        get_logger(),
        "RGB-D sync still loose: abs diff avg/p95/max %.1f/%.1f/%.1f ms, color/depth %.1f/%.1f Hz",
        pair.avg, pair.p95, pair.max, color_hz, depth_hz);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "RGB-D sync abs diff avg/p95/max %.1f/%.1f/%.1f ms, color/depth %.1f/%.1f Hz",
        pair.avg, pair.p95, pair.max, color_hz, depth_hz);
    }
  }

  std::string color_topic_;
  std::string depth_topic_;
  std::string stats_topic_;
  double max_pair_interval_ms_{40.0};
  double warn_p95_ms_{25.0};

  std::mutex mutex_;
  std::deque<int64_t> color_queue_;
  std::deque<int64_t> depth_queue_;
  int64_t last_color_stamp_ns_{0};
  int64_t last_depth_stamp_ns_{0};
  double latest_signed_diff_ms_{0.0};
  std::unique_ptr<MetricWindow> pair_diff_window_;
  std::unique_ptr<MetricWindow> color_gap_window_;
  std::unique_ptr<MetricWindow> depth_gap_window_;

  uint64_t color_received_{0U};
  uint64_t depth_received_{0U};
  uint64_t paired_frames_{0U};
  uint64_t color_unmatched_{0U};
  uint64_t depth_unmatched_{0U};
  uint64_t color_duplicate_{0U};
  uint64_t depth_duplicate_{0U};
  uint64_t color_backward_{0U};
  uint64_t depth_backward_{0U};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stats_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RgbdTimestampMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
