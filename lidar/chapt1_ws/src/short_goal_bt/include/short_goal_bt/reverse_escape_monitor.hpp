#ifndef SHORT_GOAL_BT__REVERSE_ESCAPE_MONITOR_HPP_
#define SHORT_GOAL_BT__REVERSE_ESCAPE_MONITOR_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/condition_node.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace short_goal_bt
{

class ReverseEscapeMonitor : public BT::StatefulActionNode
{
public:
  ReverseEscapeMonitor(
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("cmd_vel_topic", "/cmd_vel", "Velocity command topic"),
      BT::InputPort<double>("reverse_velocity_threshold", 0.01, "Reverse detection speed"),
      BT::InputPort<double>("stopped_velocity_threshold", 0.005, "Reverse-finished speed"),
      BT::InputPort<double>("min_reverse_duration", 0.30, "Minimum reverse duration"),
      BT::InputPort<double>("min_reverse_distance", 0.08, "Minimum reverse displacement"),
      BT::InputPort<double>("nonreverse_hold_time", 0.20, "Non-reverse hold time"),
      BT::InputPort<double>(
        "reverse_start_window", 1.0,
        "Only accept reverse starting this soon after NoShim starts"),
      BT::InputPort<std::string>("fixed_frame", "odom", "Fixed frame for displacement"),
      BT::InputPort<std::string>("robot_base_frame", "base_link", "Robot base frame"),
      BT::OutputPort<bool>("completed", "True only after a real reverse escape")
    };
  }

private:
  BT::NodeStatus evaluate();
  bool getRobotPosition(
    const std::string & fixed_frame,
    const std::string & robot_base_frame,
    double & x,
    double & y);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_executor_;
  std::mutex twist_mutex_;
  geometry_msgs::msg::Twist latest_twist_;
  bool have_twist_{false};
  bool reverse_seen_{false};
  bool reverse_window_expired_{false};
  bool nonreverse_hold_started_{false};
  rclcpp::Time monitor_start_;
  rclcpp::Time reverse_start_;
  rclcpp::Time nonreverse_start_;
  double reverse_start_x_{0.0};
  double reverse_start_y_{0.0};
  double max_reverse_distance_{0.0};
};

class ControllerSelectedCondition : public BT::ConditionNode
{
public:
  ControllerSelectedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf) {}

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("selected_controller", "Selected controller ID"),
      BT::InputPort<std::string>("expected_controller", "FollowPathNoShim", "Expected ID")
    };
  }
};

class ReverseEscapeCompletedCondition : public BT::ConditionNode
{
public:
  ReverseEscapeCompletedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf) {}

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<bool>("completed", false, "Reverse escape completion flag")};
  }
};

}  // namespace short_goal_bt

#endif  // SHORT_GOAL_BT__REVERSE_ESCAPE_MONITOR_HPP_
