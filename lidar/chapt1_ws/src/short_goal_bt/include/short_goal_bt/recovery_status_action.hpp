#pragma once

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace short_goal_bt
{

class RecoveryStatusAction : public BT::SyncActionNode
{
public:
  RecoveryStatusAction(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("stage", "tracking", "Recovery stage"),
      BT::InputPort<std::string>("reason", "", "Recovery trigger reason")};
  }

  BT::NodeStatus tick() override;

private:
  static std::string jsonEscape(const std::string & value);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::string last_payload_;
};

}  // namespace short_goal_bt
