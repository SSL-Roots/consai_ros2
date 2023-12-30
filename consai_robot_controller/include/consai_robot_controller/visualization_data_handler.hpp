// Copyright 2023 Roots
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

#ifndef CONSAI_ROBOT_CONTROLLER__VISUALIZATION_DATA_HANDLER_HPP_
#define CONSAI_ROBOT_CONTROLLER__VISUALIZATION_DATA_HANDLER_HPP_

#include <memory>

#include "consai_msgs/msg/goal_pose.hpp"
#include "consai_visualizer_msgs/msg/objects.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robocup_ssl_msgs/msg/tracked_robot.hpp"

namespace consai_robot_controller
{

using GoalPose = consai_msgs::msg::GoalPose;
using VisualizerObjects = consai_visualizer_msgs::msg::Objects;
using TrackedRobot = robocup_ssl_msgs::msg::TrackedRobot;

class VisualizationDataHandler
{
public:
  explicit VisualizationDataHandler(const rclcpp::Publisher<VisualizerObjects>::SharedPtr ptr);
  ~VisualizationDataHandler() = default;

  bool append_vis_goal(
    const TrackedRobot & robot,
    const GoalPose & goal_pose, const GoalPose & final_goal_pose,
    const GoalPose & virtual_target_pose);
  void publish_and_reset_vis_goal(void);

private:
  rclcpp::Publisher<VisualizerObjects>::SharedPtr pub_vis_objects_;
  std::unique_ptr<VisualizerObjects> vis_objects_goal_;
};

}  // namespace consai_robot_controller

#endif  // CONSAI_ROBOT_CONTROLLER__VISUALIZATION_DATA_HANDLER_HPP_
