// Copyright (c) 2021 Joshua Wallace
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_ROBOT_IN_GOAL_VICINITY_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_ROBOT_IN_GOAL_VICINITY_CONDITION_HPP_
#include <chrono>
#include <string>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ConditionNode that returns SUCCESS when the IsPathValid
 * service returns true and FAILURE otherwise
 */
class IsRobotInGoalVicinityCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::IsRobotInGoalVicinity
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  IsRobotInGoalVicinityCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsRobotInGoalVicinityCondition() = delete;

  // Destructor

  ~IsRobotInGoalVicinityCondition() override;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Function to read parameters and initialize class variables
   */
  void initialize();

  /**
   * @brief Checks if the current robot pose lies within a given distance from the goal
   * @return bool true when goal is reached, false otherwise
   */
  bool isGoalReached();

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Path to Check"),
      BT::InputPort<std::string>("robot_base_frame", std::string("base_link"), "Robot base frame"),
      BT::InputPort<std::string>("global_frame", std::string("map"), "global frame")
    };
  }

protected:
  /**
   * @brief Cleanup function
   */
  void cleanup()
  {}

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav_msgs::msg::Path path_;
  std::string global_frame_;
  std::string robot_base_frame_;

  bool initialized_;
  double goal_vic_tol_;
  double orient_angle_rad_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_ROBOT_IN_GOAL_VICINITY_CONDITION_HPP_

