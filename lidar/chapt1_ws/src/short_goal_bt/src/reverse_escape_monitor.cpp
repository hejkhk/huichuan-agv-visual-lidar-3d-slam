#include "short_goal_bt/reverse_escape_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "tf2/time.hpp"

namespace short_goal_bt
{

ReverseEscapeMonitor::ReverseEscapeMonitor(
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(action_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

  std::string cmd_vel_topic = "/cmd_vel";
  getInput("cmd_vel_topic", cmd_vel_topic);
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic, rclcpp::QoS(rclcpp::KeepLast(1)),
    [this](geometry_msgs::msg::Twist::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(twist_mutex_);
      latest_twist_ = *msg;
      have_twist_ = true;
    }, options);
  callback_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());
}

BT::NodeStatus ReverseEscapeMonitor::onStart()
{
  {
    std::lock_guard<std::mutex> lock(twist_mutex_);
    latest_twist_ = geometry_msgs::msg::Twist();
    have_twist_ = false;
  }
  reverse_seen_ = false;
  reverse_window_expired_ = false;
  nonreverse_hold_started_ = false;
  max_reverse_distance_ = 0.0;
  monitor_start_ = node_->now();
  setOutput("completed", false);
  return evaluate();
}

BT::NodeStatus ReverseEscapeMonitor::onRunning()
{
  return evaluate();
}

void ReverseEscapeMonitor::onHalted()
{
  reverse_seen_ = false;
  reverse_window_expired_ = false;
  nonreverse_hold_started_ = false;
  max_reverse_distance_ = 0.0;
  setOutput("completed", false);
}

bool ReverseEscapeMonitor::getRobotPosition(
  const std::string & fixed_frame,
  const std::string & robot_base_frame,
  double & x,
  double & y)
{
  try {
    const auto transform = tf_->lookupTransform(
      fixed_frame, robot_base_frame, tf2::TimePointZero,
      tf2::durationFromSec(0.1));
    x = transform.transform.translation.x;
    y = transform.transform.translation.y;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "Reverse escape monitor TF unavailable: %s", ex.what());
    return false;
  }
}

BT::NodeStatus ReverseEscapeMonitor::evaluate()
{
  callback_executor_.spin_some();

  double reverse_velocity_threshold = 0.01;
  double stopped_velocity_threshold = 0.005;
  double min_reverse_duration = 0.30;
  double min_reverse_distance = 0.08;
  double nonreverse_hold_time = 0.20;
  double reverse_start_window = 1.0;
  std::string fixed_frame = "odom";
  std::string robot_base_frame = "base_link";
  getInput("reverse_velocity_threshold", reverse_velocity_threshold);
  getInput("stopped_velocity_threshold", stopped_velocity_threshold);
  getInput("min_reverse_duration", min_reverse_duration);
  getInput("min_reverse_distance", min_reverse_distance);
  getInput("nonreverse_hold_time", nonreverse_hold_time);
  getInput("reverse_start_window", reverse_start_window);
  getInput("fixed_frame", fixed_frame);
  getInput("robot_base_frame", robot_base_frame);

  if (reverse_velocity_threshold <= 0.0 || stopped_velocity_threshold < 0.0 ||
    stopped_velocity_threshold >= reverse_velocity_threshold || min_reverse_duration < 0.0 ||
    min_reverse_distance < 0.0 || nonreverse_hold_time < 0.0 || reverse_start_window <= 0.0)
  {
    RCLCPP_ERROR(node_->get_logger(), "ReverseEscapeMonitor has invalid thresholds");
    return BT::NodeStatus::FAILURE;
  }

  double velocity_x = 0.0;
  {
    std::lock_guard<std::mutex> lock(twist_mutex_);
    if (!have_twist_) {
      return BT::NodeStatus::RUNNING;
    }
    velocity_x = latest_twist_.linear.x;
  }

  const auto now = node_->now();
  if (!reverse_seen_ && (now - monitor_start_).seconds() > reverse_start_window) {
    if (!reverse_window_expired_) {
      reverse_window_expired_ = true;
      RCLCPP_INFO(
        node_->get_logger(),
        "Reverse escape start window expired after %.2f s; later reverse commands "
        "will be treated as normal DWB control",
        reverse_start_window);
    }
    return BT::NodeStatus::RUNNING;
  }

  if (velocity_x <= -reverse_velocity_threshold) {
    nonreverse_hold_started_ = false;
    if (!reverse_seen_) {
      if (!getRobotPosition(fixed_frame, robot_base_frame, reverse_start_x_, reverse_start_y_)) {
        return BT::NodeStatus::RUNNING;
      }
      reverse_seen_ = true;
      reverse_start_ = now;
      RCLCPP_INFO(
        node_->get_logger(),
        "Reverse escape detected at %.3f m/s; monitoring for completion",
        velocity_x);
    }
  }

  if (!reverse_seen_) {
    return BT::NodeStatus::RUNNING;
  }

  double robot_x = 0.0;
  double robot_y = 0.0;
  if (getRobotPosition(fixed_frame, robot_base_frame, robot_x, robot_y)) {
    max_reverse_distance_ = std::max(
      max_reverse_distance_,
      std::hypot(robot_x - reverse_start_x_, robot_y - reverse_start_y_));
  }

  if (velocity_x > -stopped_velocity_threshold) {
    if (!nonreverse_hold_started_) {
      nonreverse_hold_started_ = true;
      nonreverse_start_ = now;
    }
  } else {
    nonreverse_hold_started_ = false;
  }

  const double reverse_duration = (now - reverse_start_).seconds();
  const double nonreverse_duration = nonreverse_hold_started_ ?
    (now - nonreverse_start_).seconds() : 0.0;
  if (reverse_duration >= min_reverse_duration &&
    max_reverse_distance_ >= min_reverse_distance &&
    nonreverse_duration >= nonreverse_hold_time)
  {
    setOutput("completed", true);
    RCLCPP_INFO(
      node_->get_logger(),
      "Reverse escape completed after %.2f m and %.2f s; stopping NoShim for path realignment",
      max_reverse_distance_, reverse_duration);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ControllerSelectedCondition::tick()
{
  std::string selected;
  std::string expected = "FollowPathNoShim";
  if (!getInput("selected_controller", selected)) {
    return BT::NodeStatus::FAILURE;
  }
  getInput("expected_controller", expected);
  return selected == expected ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus ReverseEscapeCompletedCondition::tick()
{
  bool completed = false;
  getInput("completed", completed);
  return completed ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace short_goal_bt
