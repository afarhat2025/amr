#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__REPLAN_TRIGGER_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__REPLAN_TRIGGER_CONDITION_HPP_

#include <string>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "std_msgs/msg/bool.hpp"

namespace nav2_behavior_tree
{

class ReplanTriggerCondition : public BT::ConditionNode
{
public:
  ReplanTriggerCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  ~ReplanTriggerCondition() override;
  void replanCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void logReplanStatus(const std::string & msg) const;
  BT::NodeStatus tick() override;

  // Define input ports for the topic name
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("topic_name", "Topic to subscribe for replan trigger")};
  }

private:
  // The node that will be used for any ROS operations
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::thread callback_group_executor_thread;

  // Flag indicating if replan is triggered
  std::atomic<bool> is_replan_triggered_;

  // Subscription to replan trigger topic
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr replan_sub_;

  // Replan trigger topic
  std::string topic_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__REPLAN_TRIGGER_CONDITION_HPP_