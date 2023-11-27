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

#include "consai_msgs/msg/goal_poses.hpp"
#include "consai_visualizer_msgs/msg/objects.hpp"
#include "rclcpp/rclcpp.hpp"

namespace consai_robot_controller
{

using GoalPoses = consai_msgs::msg::GoalPoses;
using VisualizerObjects = consai_visualizer_msgs::msg::Objects;

class VisualizationDataHandler
{
 public:
  explicit VisualizationDataHandler(const rclcpp::Publisher<VisualizerObjects>::SharedPtr ptr);
  ~VisualizationDataHandler() = default;

  void publish_vis_goal(
    const GoalPoses::SharedPtr goal_poses, const GoalPoses::SharedPtr final_goal_poses);

 private:
  rclcpp::Publisher<VisualizerObjects>::SharedPtr pub_vis_objects_;
};

}  // namespace consai_robot_controller

#endif  // CONSAI_ROBOT_CONTROLLER__VISUALIZATION_DATA_HANDLER_HPP_
