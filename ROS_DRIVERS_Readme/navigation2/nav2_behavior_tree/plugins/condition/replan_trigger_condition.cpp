

#include <string>
#include <chrono>

#include "nav2_behavior_tree/plugins/condition/replan_trigger_condition.hpp"

using namespace std::chrono_literals; // NOLINT

namespace nav2_behavior_tree
{

ReplanTriggerCondition::ReplanTriggerCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  is_replan_triggered_(false)
{
  if (!getInput<std::string>("topic_name", topic_)) {
    throw BT::RuntimeError("Missing required parameter 'topic_name' for ReplanTriggerCondition");
  }

  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  callback_group_executor_thread = std::thread([this]() { callback_group_executor_.spin(); });


  // Set up the subscription to the replan trigger topic
  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  replan_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&ReplanTriggerCondition::replanCallback, this, std::placeholders::_1),
    sub_option);

  RCLCPP_DEBUG(node_->get_logger(), "Initialized ReplanTriggerCondition BT node");
}

ReplanTriggerCondition::~ReplanTriggerCondition()
{
  RCLCPP_DEBUG(node_->get_logger(), "Shutting down ReplanTriggerCondition BT node");
  callback_group_executor_.cancel();
  callback_group_executor_thread.join();
}

void ReplanTriggerCondition::replanCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    is_replan_triggered_ = true;
    logReplanStatus("Replan trigger received.");
  } else {
    logReplanStatus("Replan trigger cleared.");
  }
}

BT::NodeStatus ReplanTriggerCondition::tick()
{
  if (is_replan_triggered_) {
    is_replan_triggered_ = false; 
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

void ReplanTriggerCondition::logReplanStatus(const std::string & msg) const
{
  RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::ReplanTriggerCondition>("ReplanTrigger");
}