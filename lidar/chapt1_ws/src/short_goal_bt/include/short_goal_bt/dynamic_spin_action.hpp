#pragma once

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/spin.hpp"

namespace short_goal_bt
{

// Humble's built-in Spin node reads spin_dist in its constructor, before a
// preceding BT node can write a computed angle to the blackboard. This variant
// reads every input when ticked so runtime-computed pre-rotation angles are used.
class DynamicSpinAction
  : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::Spin>
{
public:
  DynamicSpinAction(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<double>("spin_dist", "Runtime spin angle in radians"),
        BT::InputPort<double>("time_allowance", 12.0, "Allowed spin time"),
        BT::InputPort<bool>("is_recovery", false, "Count as a recovery action")
      });
  }

  void on_tick() override;
};

}  // namespace short_goal_bt
