#ifndef SHORT_GOAL_BT__SHORT_GOAL_BEHIND_CONDITION_HPP_
#define SHORT_GOAL_BT__SHORT_GOAL_BEHIND_CONDITION_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace short_goal_bt
{

class ShortGoalBehindCondition : public BT::ConditionNode
{
public:
  ShortGoalBehindCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Navigation goal"),
      BT::InputPort<double>("min_distance", 0.30, "Minimum goal distance in metres"),
      BT::InputPort<double>("max_distance", 1.20, "Maximum goal distance in metres"),
      BT::InputPort<double>("min_bearing", 2.09439510239, "Rear bearing threshold in radians"),
      BT::InputPort<std::string>("robot_base_frame", "base_link", "Robot base frame"),
      BT::OutputPort<double>("spin_dist", "Signed angle to rotate toward the goal")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
};

}  // namespace short_goal_bt

#endif  // SHORT_GOAL_BT__SHORT_GOAL_BEHIND_CONDITION_HPP_
