#include "short_goal_bt/spin_safety_condition.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

#include "tf2/time.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace short_goal_bt
{

SpinSafetyCondition::SpinSafetyCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  costmap_callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = costmap_callback_group_;
  local_costmap_sub_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
    "/local_costmap/costmap_raw", qos,
    [this](nav2_msgs::msg::Costmap::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      local_costmap_ = std::move(msg);
    }, subscription_options);
  global_costmap_sub_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
    "/global_costmap/costmap_raw", qos,
    [this](nav2_msgs::msg::Costmap::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      global_costmap_ = std::move(msg);
    }, subscription_options);
  costmap_executor_.add_callback_group(
    costmap_callback_group_, node_->get_node_base_interface());
}

bool SpinSafetyCondition::costmapIsClear(
  const nav2_msgs::msg::Costmap & costmap,
  const double safety_radius,
  const int lethal_cost,
  const bool block_unknown,
  const std::string & robot_base_frame)
{
  const auto size_x = costmap.metadata.size_x;
  const auto size_y = costmap.metadata.size_y;
  const double resolution = costmap.metadata.resolution;
  if (size_x == 0 || size_y == 0 || resolution <= 0.0 ||
    costmap.data.size() != static_cast<std::size_t>(size_x) * size_y)
  {
    RCLCPP_WARN(node_->get_logger(), "Spin safety received an invalid costmap");
    return false;
  }

  geometry_msgs::msg::TransformStamped robot_tf;
  try {
    robot_tf = tf_->lookupTransform(
      costmap.header.frame_id, robot_base_frame, tf2::TimePointZero,
      tf2::durationFromSec(0.2));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "Spin safety TF unavailable for %s: %s", costmap.header.frame_id.c_str(), ex.what());
    return false;
  }

  const double origin_yaw = tf2::getYaw(costmap.metadata.origin.orientation);
  const double cosine = std::cos(origin_yaw);
  const double sine = std::sin(origin_yaw);
  const double world_dx =
    robot_tf.transform.translation.x - costmap.metadata.origin.position.x;
  const double world_dy =
    robot_tf.transform.translation.y - costmap.metadata.origin.position.y;
  const double robot_map_x = cosine * world_dx + sine * world_dy;
  const double robot_map_y = -sine * world_dx + cosine * world_dy;

  const int center_x = static_cast<int>(std::floor(robot_map_x / resolution));
  const int center_y = static_cast<int>(std::floor(robot_map_y / resolution));
  const int radius_cells = static_cast<int>(std::ceil(safety_radius / resolution));
  const int min_x = std::max(0, center_x - radius_cells);
  const int max_x = std::min(static_cast<int>(size_x) - 1, center_x + radius_cells);
  const int min_y = std::max(0, center_y - radius_cells);
  const int max_y = std::min(static_cast<int>(size_y) - 1, center_y + radius_cells);
  const double radius_squared = safety_radius * safety_radius;

  for (int y = min_y; y <= max_y; ++y) {
    for (int x = min_x; x <= max_x; ++x) {
      const double cell_x = (static_cast<double>(x) + 0.5) * resolution;
      const double cell_y = (static_cast<double>(y) + 0.5) * resolution;
      const double dx = cell_x - robot_map_x;
      const double dy = cell_y - robot_map_y;
      if (dx * dx + dy * dy > radius_squared) {
        continue;
      }

      const uint8_t cost = costmap.data[static_cast<std::size_t>(y) * size_x + x];
      const bool unknown_blocked = block_unknown && cost == 255;
      const bool lethal_blocked = cost != 255 && cost >= lethal_cost;
      if (unknown_blocked || lethal_blocked) {
        RCLCPP_WARN(
          node_->get_logger(),
          "Spin safety blocked by %s costmap: cost %u at %.2f m within %.2f m radius",
          costmap.header.frame_id.c_str(), static_cast<unsigned int>(cost),
          std::hypot(dx, dy), safety_radius);
        return false;
      }
    }
  }
  return true;
}

BT::NodeStatus SpinSafetyCondition::onStart()
{
  wait_start_ = node_->now();
  return evaluate();
}

BT::NodeStatus SpinSafetyCondition::onRunning()
{
  return evaluate();
}

BT::NodeStatus SpinSafetyCondition::evaluate()
{
  costmap_executor_.spin_some();

  double safety_radius = 0.54;
  double costmap_wait_timeout = 1.0;
  int lethal_cost = 254;
  bool block_unknown = true;
  std::string robot_base_frame = "base_link";
  getInput("safety_radius", safety_radius);
  getInput("lethal_cost", lethal_cost);
  getInput("block_unknown", block_unknown);
  getInput("robot_base_frame", robot_base_frame);
  getInput("costmap_wait_timeout", costmap_wait_timeout);

  if (safety_radius <= 0.0 || lethal_cost < 0 || lethal_cost > 255 ||
    costmap_wait_timeout < 0.0)
  {
    RCLCPP_ERROR(node_->get_logger(), "SpinSafetyCheck has invalid parameters");
    return BT::NodeStatus::FAILURE;
  }

  nav2_msgs::msg::Costmap::SharedPtr local;
  nav2_msgs::msg::Costmap::SharedPtr global;
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    local = local_costmap_;
    global = global_costmap_;
  }
  if (!local || !global) {
    const double elapsed = (node_->now() - wait_start_).seconds();
    if (elapsed < costmap_wait_timeout) {
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "Spin safety is waiting for local and global costmaps (%.2f/%.2f s)",
        elapsed, costmap_wait_timeout);
      return BT::NodeStatus::RUNNING;
    }
    RCLCPP_WARN(
      node_->get_logger(),
      "Spin safety costmap wait timed out after %.2f s; using no-shim fallback",
      costmap_wait_timeout);
    return BT::NodeStatus::FAILURE;
  }

  if (!costmapIsClear(
      *local, safety_radius, lethal_cost, block_unknown, robot_base_frame) ||
    !costmapIsClear(
      *global, safety_radius, lethal_cost, block_unknown, robot_base_frame))
  {
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    node_->get_logger(), "Spin safety clear within %.2f m; allowing pre-rotation",
    safety_radius);
  return BT::NodeStatus::SUCCESS;
}

SelectControllerAction::SelectControllerAction(
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(action_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::NodeStatus SelectControllerAction::tick()
{
  std::string controller_id = "FollowPathNoShim";
  if (!getInput("controller_id", controller_id) || controller_id.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  setOutput("selected_controller", controller_id);
  if (controller_id == "FollowPathNoShim") {
    RCLCPP_WARN(
      node_->get_logger(),
      "Pre-rotation is unsafe or failed; skipping Spin and selecting controller '%s'",
      controller_id.c_str());
  } else {
    RCLCPP_INFO(
      node_->get_logger(),
      "Path alignment complete; selecting controller '%s'",
      controller_id.c_str());
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace short_goal_bt
