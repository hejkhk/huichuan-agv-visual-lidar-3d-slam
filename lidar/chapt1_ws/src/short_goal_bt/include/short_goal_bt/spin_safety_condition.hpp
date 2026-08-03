#ifndef SHORT_GOAL_BT__SPIN_SAFETY_CONDITION_HPP_
#define SHORT_GOAL_BT__SPIN_SAFETY_CONDITION_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_msgs/msg/costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "tf2_ros/buffer.h"

namespace short_goal_bt
{

class SpinSafetyCondition : public BT::StatefulActionNode
{
public:
  SpinSafetyCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("safety_radius", 0.54, "Full-turn swept radius in metres"),
      BT::InputPort<int>("lethal_cost", 254, "Minimum obstacle cost that blocks spinning"),
      BT::InputPort<bool>("block_unknown", true, "Treat unknown costmap cells as blocked"),
      BT::InputPort<std::string>("robot_base_frame", "base_link", "Robot base frame"),
      BT::InputPort<double>("costmap_wait_timeout", 1.0, "Seconds to wait for costmaps")
    };
  }

private:
  BT::NodeStatus evaluate();

  bool costmapIsClear(
    const nav2_msgs::msg::Costmap & costmap,
    double safety_radius,
    int lethal_cost,
    bool block_unknown,
    const std::string & robot_base_frame);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr local_costmap_sub_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr global_costmap_sub_;
  rclcpp::CallbackGroup::SharedPtr costmap_callback_group_;
  rclcpp::executors::SingleThreadedExecutor costmap_executor_;
  std::mutex costmap_mutex_;
  nav2_msgs::msg::Costmap::SharedPtr local_costmap_;
  nav2_msgs::msg::Costmap::SharedPtr global_costmap_;
  rclcpp::Time wait_start_;
};

class SelectControllerAction : public BT::SyncActionNode
{
public:
  SelectControllerAction(
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("controller_id", "FollowPathNoShim", "Controller to select"),
      BT::OutputPort<std::string>("selected_controller", "Selected controller ID")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace short_goal_bt

#endif  // SHORT_GOAL_BT__SPIN_SAFETY_CONDITION_HPP_
