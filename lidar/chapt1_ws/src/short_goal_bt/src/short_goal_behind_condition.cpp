#include "short_goal_bt/dynamic_spin_action.hpp"
#include "short_goal_bt/short_goal_behind_condition.hpp"
#include "short_goal_bt/initial_path_pre_rotate_condition.hpp"
#include "short_goal_bt/reverse_escape_monitor.hpp"
#include "short_goal_bt/recovery_status_action.hpp"
#include "short_goal_bt/spin_safety_condition.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "tf2/time.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace short_goal_bt
{

ShortGoalBehindCondition::ShortGoalBehindCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
}

BT::NodeStatus ShortGoalBehindCondition::tick()
{
  geometry_msgs::msg::PoseStamped goal;
  double min_distance = 0.30;
  double max_distance = 1.20;
  double min_bearing = 2.09439510239;
  std::string robot_base_frame = "base_link";

  if (!getInput("goal", goal) || goal.header.frame_id.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  getInput("min_distance", min_distance);
  getInput("max_distance", max_distance);
  getInput("min_bearing", min_bearing);
  getInput("robot_base_frame", robot_base_frame);

  if (min_distance < 0.0 || max_distance <= min_distance ||
    min_bearing <= 0.0 || min_bearing > M_PI)
  {
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "ShortGoalBehind has invalid distance or bearing thresholds");
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::TransformStamped robot_tf;
  try {
    robot_tf = tf_->lookupTransform(
      goal.header.frame_id, robot_base_frame, tf2::TimePointZero,
      tf2::durationFromSec(0.2));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "ShortGoalBehind TF unavailable: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }

  const double dx = goal.pose.position.x - robot_tf.transform.translation.x;
  const double dy = goal.pose.position.y - robot_tf.transform.translation.y;
  const double distance = std::hypot(dx, dy);
  if (distance < min_distance || distance > max_distance) {
    return BT::NodeStatus::FAILURE;
  }

  const double robot_yaw = tf2::getYaw(robot_tf.transform.rotation);
  const double goal_bearing = std::atan2(dy, dx);
  const double bearing_error = std::atan2(
    std::sin(goal_bearing - robot_yaw),
    std::cos(goal_bearing - robot_yaw));

  if (std::abs(bearing_error) < min_bearing) {
    return BT::NodeStatus::FAILURE;
  }

  setOutput("spin_dist", bearing_error);
  RCLCPP_INFO(
    node_->get_logger(),
    "Short rear goal detected: distance %.2f m, bearing %.1f deg; pre-rotating in place",
    distance, bearing_error * 180.0 / M_PI);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace short_goal_bt

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<short_goal_bt::DynamicSpinAction>("DynamicSpin");
  factory.registerNodeType<short_goal_bt::ShortGoalBehindCondition>("ShortGoalBehind");
  factory.registerNodeType<short_goal_bt::InitialPathPreRotateCondition>("InitialPathPreRotate");
  factory.registerNodeType<short_goal_bt::SpinSafetyCondition>("SpinSafetyCheck");
  factory.registerNodeType<short_goal_bt::SelectControllerAction>("SelectController");
  factory.registerNodeType<short_goal_bt::ReverseEscapeMonitor>("ReverseEscapeMonitor");
  factory.registerNodeType<short_goal_bt::ControllerSelectedCondition>("ControllerSelected");
  factory.registerNodeType<short_goal_bt::ReverseEscapeCompletedCondition>(
    "ReverseEscapeCompleted");
  factory.registerNodeType<short_goal_bt::RecoveryStatusAction>("RecoveryStatus");
}
