#ifndef SHORT_GOAL_BT__INITIAL_PATH_PRE_ROTATE_CONDITION_HPP_
#define SHORT_GOAL_BT__INITIAL_PATH_PRE_ROTATE_CONDITION_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_cpp/json_export.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/json_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace short_goal_bt
{

// Computes a one-shot in-place rotation before FollowPath starts for each new goal.
// Short rear goals rotate toward the goal. Longer paths rotate toward the first
// intersection between the path and a circle enclosing the square footprint.
class InitialPathPreRotateCondition : public BT::ConditionNode
{
public:
  InitialPathPreRotateCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    BT::RegisterJsonDefinition<geometry_msgs::msg::PoseStamped>();
    BT::RegisterJsonDefinition<nav_msgs::msg::Path>();
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Navigation goal"),
      BT::InputPort<nav_msgs::msg::Path>("path", "Newly computed global path"),
      BT::InputPort<double>("min_short_distance", 0.30, "Minimum short-goal distance"),
      BT::InputPort<double>("max_short_distance", 1.20, "Maximum short-goal distance"),
      BT::InputPort<double>("min_short_bearing", 2.09439510239, "Short rear-goal angle"),
      BT::InputPort<double>("min_path_length", 1.20, "Minimum long path length"),
      BT::InputPort<double>("circle_radius", 0.50911688245, "Square footprint circumradius"),
      BT::InputPort<double>("min_path_bearing", 0.20, "Long-path rotation deadband"),
      BT::InputPort<std::string>("robot_base_frame", "base_link", "Robot base frame"),
      BT::OutputPort<double>("spin_dist", "Signed pre-rotation angle")
    };
  }

private:
  bool isNewGoal(const geometry_msgs::msg::PoseStamped & goal) const;
  void rememberGoal(const geometry_msgs::msg::PoseStamped & goal);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  geometry_msgs::msg::PoseStamped remembered_goal_;
  bool have_goal_{false};
  bool evaluated_for_goal_{false};
};

}  // namespace short_goal_bt

#endif  // SHORT_GOAL_BT__INITIAL_PATH_PRE_ROTATE_CONDITION_HPP_
