#include "short_goal_bt/dynamic_spin_action.hpp"

#include <cmath>

#include "rclcpp/duration.hpp"

namespace short_goal_bt
{

DynamicSpinAction::DynamicSpinAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: nav2_behavior_tree::BtActionNode<nav2_msgs::action::Spin>(name, "spin", config)
{
}

void DynamicSpinAction::on_tick()
{
  double spin_dist = 0.0;
  double time_allowance = 12.0;
  bool is_recovery = false;

  const auto spin_result = getInput("spin_dist", spin_dist);
  if (!spin_result) {
    RCLCPP_ERROR(
      node_->get_logger(), "DynamicSpin has no runtime spin_dist: %s",
      spin_result.error().c_str());
    should_send_goal_ = false;
    return;
  }
  getInput("time_allowance", time_allowance);
  getInput("is_recovery", is_recovery);

  if (!std::isfinite(spin_dist) || std::abs(spin_dist) < 1.0e-3 ||
    !std::isfinite(time_allowance) || time_allowance <= 0.0)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "DynamicSpin rejected invalid command: angle=%.6f rad allowance=%.3f s",
      spin_dist, time_allowance);
    should_send_goal_ = false;
    return;
  }

  goal_.target_yaw = spin_dist;
  goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
  if (is_recovery) {
    increment_recovery_count();
  }
  RCLCPP_INFO(
    node_->get_logger(), "DynamicSpin sending %.1f deg (allowance %.1f s)",
    spin_dist * 180.0 / 3.14159265358979323846, time_allowance);
}

}  // namespace short_goal_bt
