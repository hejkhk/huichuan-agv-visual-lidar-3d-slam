#include "short_goal_bt/recovery_status_action.hpp"

#include <sstream>

namespace short_goal_bt
{

RecoveryStatusAction::RecoveryStatusAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  publisher_ = node_->create_publisher<std_msgs::msg::String>(
    "/navigation/recovery_status", qos);
}

std::string RecoveryStatusAction::jsonEscape(const std::string & value)
{
  std::ostringstream output;
  for (const char character : value) {
    switch (character) {
      case '\\': output << "\\\\"; break;
      case '"': output << "\\\""; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default: output << character; break;
    }
  }
  return output.str();
}

BT::NodeStatus RecoveryStatusAction::tick()
{
  std::string stage = "tracking";
  std::string reason;
  getInput("stage", stage);
  getInput("reason", reason);

  const std::string payload =
    "{\"stage\":\"" + jsonEscape(stage) +
    "\",\"reason\":\"" + jsonEscape(reason) + "\"}";
  if (payload != last_payload_) {
    std_msgs::msg::String message;
    message.data = payload;
    publisher_->publish(message);
    last_payload_ = payload;
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace short_goal_bt
