#include "short_goal_bt/initial_path_pre_rotate_condition.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "tf2/time.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace short_goal_bt
{

namespace
{

double normalizeAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double pathLength(const nav_msgs::msg::Path & path)
{
  double length = 0.0;
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    const auto & previous = path.poses[i - 1].pose.position;
    const auto & current = path.poses[i].pose.position;
    length += std::hypot(current.x - previous.x, current.y - previous.y);
  }
  return length;
}

bool firstCircleIntersection(
  const nav_msgs::msg::Path & path,
  const double center_x,
  const double center_y,
  const double radius,
  double & intersection_x,
  double & intersection_y)
{
  constexpr double epsilon = 1e-9;
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    const auto & start = path.poses[i - 1].pose.position;
    const auto & end = path.poses[i].pose.position;
    const double segment_x = end.x - start.x;
    const double segment_y = end.y - start.y;
    const double from_center_x = start.x - center_x;
    const double from_center_y = start.y - center_y;
    const double a = segment_x * segment_x + segment_y * segment_y;
    if (a < epsilon) {
      continue;
    }

    const double b = 2.0 *
      (from_center_x * segment_x + from_center_y * segment_y);
    const double c = from_center_x * from_center_x +
      from_center_y * from_center_y - radius * radius;
    const double discriminant = b * b - 4.0 * a * c;
    if (discriminant < 0.0) {
      continue;
    }

    const double root = std::sqrt(std::max(0.0, discriminant));
    const double first = (-b - root) / (2.0 * a);
    const double second = (-b + root) / (2.0 * a);
    const double candidates[2] = {first, second};
    for (const double t : candidates) {
      if (t >= -epsilon && t <= 1.0 + epsilon) {
        const double clamped_t = std::clamp(t, 0.0, 1.0);
        intersection_x = start.x + clamped_t * segment_x;
        intersection_y = start.y + clamped_t * segment_y;
        return true;
      }
    }
  }
  return false;
}

}  // namespace

InitialPathPreRotateCondition::InitialPathPreRotateCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
}

bool InitialPathPreRotateCondition::isNewGoal(
  const geometry_msgs::msg::PoseStamped & goal) const
{
  if (!have_goal_) {
    return true;
  }
  const bool stamp_changed =
    goal.header.stamp.sec != remembered_goal_.header.stamp.sec ||
    goal.header.stamp.nanosec != remembered_goal_.header.stamp.nanosec;
  const double position_change = std::hypot(
    goal.pose.position.x - remembered_goal_.pose.position.x,
    goal.pose.position.y - remembered_goal_.pose.position.y);
  const double yaw_change = std::abs(normalizeAngle(
    tf2::getYaw(goal.pose.orientation) -
    tf2::getYaw(remembered_goal_.pose.orientation)));
  return stamp_changed || goal.header.frame_id != remembered_goal_.header.frame_id ||
         position_change > 0.01 || yaw_change > 0.01;
}

void InitialPathPreRotateCondition::rememberGoal(
  const geometry_msgs::msg::PoseStamped & goal)
{
  remembered_goal_ = goal;
  have_goal_ = true;
  evaluated_for_goal_ = false;
}

BT::NodeStatus InitialPathPreRotateCondition::tick()
{
  geometry_msgs::msg::PoseStamped goal;
  nav_msgs::msg::Path path;
  double min_short_distance = 0.30;
  double max_short_distance = 1.20;
  double min_short_bearing = 2.09439510239;
  double min_path_length = 1.20;
  double circle_radius = 0.50911688245;
  double min_path_bearing = 0.20;
  std::string robot_base_frame = "base_link";

  if (!getInput("goal", goal) || !getInput("path", path) ||
    goal.header.frame_id.empty() || path.header.frame_id.empty() || path.poses.size() < 2)
  {
    return BT::NodeStatus::FAILURE;
  }
  getInput("min_short_distance", min_short_distance);
  getInput("max_short_distance", max_short_distance);
  getInput("min_short_bearing", min_short_bearing);
  getInput("min_path_length", min_path_length);
  getInput("circle_radius", circle_radius);
  getInput("min_path_bearing", min_path_bearing);
  getInput("robot_base_frame", robot_base_frame);

  if (min_short_distance < 0.0 || max_short_distance <= min_short_distance ||
    min_short_bearing <= 0.0 || min_short_bearing > M_PI ||
    min_path_length <= 0.0 || circle_radius <= 0.0 ||
    min_path_bearing < 0.0 || min_path_bearing > M_PI)
  {
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "InitialPathPreRotate has invalid thresholds");
    return BT::NodeStatus::FAILURE;
  }

  if (isNewGoal(goal)) {
    rememberGoal(goal);
  }
  if (evaluated_for_goal_) {
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::TransformStamped robot_tf;
  try {
    robot_tf = tf_->lookupTransform(
      path.header.frame_id, robot_base_frame, tf2::TimePointZero,
      tf2::durationFromSec(0.2));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "InitialPathPreRotate TF unavailable: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }

  const double robot_x = robot_tf.transform.translation.x;
  const double robot_y = robot_tf.transform.translation.y;
  const double robot_yaw = tf2::getYaw(robot_tf.transform.rotation);
  const double goal_dx = goal.pose.position.x - robot_x;
  const double goal_dy = goal.pose.position.y - robot_y;
  const double goal_distance = std::hypot(goal_dx, goal_dy);
  const double goal_error = normalizeAngle(std::atan2(goal_dy, goal_dx) - robot_yaw);
  const double total_path_length = pathLength(path);

  evaluated_for_goal_ = true;

  if (goal_distance >= min_short_distance && goal_distance <= max_short_distance &&
    std::abs(goal_error) >= min_short_bearing)
  {
    setOutput("spin_dist", goal_error);
    RCLCPP_INFO(
      node_->get_logger(),
      "Initial short rear goal: distance %.2f m, bearing %.1f deg; pre-rotating once",
      goal_distance, goal_error * 180.0 / M_PI);
    return BT::NodeStatus::SUCCESS;
  }

  if (total_path_length <= min_path_length) {
    return BT::NodeStatus::FAILURE;
  }

  double intersection_x = 0.0;
  double intersection_y = 0.0;
  if (!firstCircleIntersection(
      path, robot_x, robot_y, circle_radius, intersection_x, intersection_y))
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "Long path is %.2f m but has no intersection with %.3f m footprint circle; skipping pre-rotation",
      total_path_length, circle_radius);
    return BT::NodeStatus::FAILURE;
  }

  const double path_error = normalizeAngle(
    std::atan2(intersection_y - robot_y, intersection_x - robot_x) - robot_yaw);
  if (std::abs(path_error) < min_path_bearing) {
    RCLCPP_INFO(
      node_->get_logger(),
      "Initial long path already aligned: length %.2f m, circle bearing %.1f deg",
      total_path_length, path_error * 180.0 / M_PI);
    return BT::NodeStatus::FAILURE;
  }

  setOutput("spin_dist", path_error);
  RCLCPP_INFO(
    node_->get_logger(),
    "Initial long path: length %.2f m, circle %.3f m, bearing %.1f deg; pre-rotating once",
    total_path_length, circle_radius, path_error * 180.0 / M_PI);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace short_goal_bt
