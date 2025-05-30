// Copyright (c) 2019 Intel Corporation
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

#include <string>
#include <memory>

#include "nav2_util/robot_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/node_utils.hpp"

#include "nav2_behavior_tree/plugins/condition/is_robot_in_goal_vicinity_condition.hpp"
//#include "nav2_behavior_tree/bt_utils.hpp"

namespace nav2_behavior_tree
{

IsRobotInGoalVicinityCondition::IsRobotInGoalVicinityCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  global_frame_("map"),
  robot_base_frame_("base_link"),
  initialized_(false)
{
  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

IsRobotInGoalVicinityCondition::~IsRobotInGoalVicinityCondition()
{
  cleanup();
}

void IsRobotInGoalVicinityCondition::initialize()
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  nav2_util::declare_parameter_if_not_declared(
    node_, "orient_angle_rad",
    rclcpp::ParameterValue(1.57));	
  nav2_util::declare_parameter_if_not_declared(
    node_, "goal_vicinity_tol",
    rclcpp::ParameterValue(2.25));
  node_->get_parameter_or<double>("goal_vicinity_tol", goal_vic_tol_, 2.25);
  node_->get_parameter_or<double>("orient_angle_rad", orient_angle_rad_, 1.57);
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  
  initialized_ = true;
}

BT::NodeStatus IsRobotInGoalVicinityCondition::tick()
{
  if (!initialized_) {
    initialize();
    //std::cout << "initialized goal vici initial" <<std::endl;
  }

  if (isGoalReached() && goal_vic_tol_ > 0.25) {
    return BT::NodeStatus::SUCCESS;
    //std::cout << "initialized goal succ" <<std::endl;
  }
  return BT::NodeStatus::FAILURE;
}
double createYawFromQuat(const geometry_msgs::msg::Quaternion & orientation)
{
  tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}
bool IsRobotInGoalVicinityCondition::isGoalReached()
{
  double transform_tolerance_ = 0.03;
  getInput("path",path_);
  nav_msgs::msg::Path & path = path_;
  geometry_msgs::msg::PoseStamped pose;
  if (!nav2_util::getCurrentPose(
      pose, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
   {
    RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
    return false;
  }
   
  auto find_closest_pose_idx =
    [this,&pose, &path]() {
      size_t closest_pose_idx = 0;
      double curr_min_dist = std::numeric_limits<double>::max();
      for (size_t curr_idx = 0; curr_idx < path.poses.size(); ++curr_idx) {
        double curr_dist = nav2_util::geometry_utils::euclidean_distance(
          pose, path.poses[curr_idx]);
        double orient_diff = createYawFromQuat(pose.pose.orientation) - createYawFromQuat(path.poses[curr_idx].pose.orientation);
        //std::cout << "angle: " << orient_angle_rad_ << std::endl;
        if (curr_dist < curr_min_dist && abs(orient_diff) < orient_angle_rad_) {
          curr_min_dist = curr_dist;
          closest_pose_idx = curr_idx;
        }
      }
      return closest_pose_idx;
    };

  double distance_remaining = nav2_util::geometry_utils::calculate_path_length(path_, find_closest_pose_idx());
  //std::cout << "initialized goal vici: " << distance_remaining <<std::endl;
  
  return distance_remaining <= goal_vic_tol_ ;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsRobotInGoalVicinityCondition>("IsRobotInGoalVicinity");
}
