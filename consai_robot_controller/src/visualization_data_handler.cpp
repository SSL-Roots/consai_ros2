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

#include "consai_robot_controller/visualization_data_handler.hpp"
#include "consai_visualizer_msgs/msg/shape_line.hpp"
#include "consai_visualizer_msgs/msg/shape_robot.hpp"

namespace consai_robot_controller
{

using VisLine = consai_visualizer_msgs::msg::ShapeLine;
using VisRobot = consai_visualizer_msgs::msg::ShapeRobot;

VisualizationDataHandler::VisualizationDataHandler(
  const rclcpp::Publisher<VisualizerObjects>::SharedPtr ptr)
: pub_vis_objects_(ptr)
{
  vis_objects_goal_ = std::make_unique<VisualizerObjects>();
}


bool VisualizationDataHandler::append_vis_goal(
  const TrackedRobot & robot,
  const GoalPose & goal_pose, const GoalPose & final_goal_pose,
  const GoalPose & virtual_target_pose)
{
  if (!vis_objects_goal_) {
    return false;
  }

  VisRobot vis_robot;
  vis_robot.line_color.name = "black";
  vis_robot.fill_color.name = "violet";
  vis_robot.fill_color.alpha = 0.8;
  vis_robot.line_size = 1;
  vis_robot.caption = "goal" + std::to_string(goal_pose.robot_id);
  vis_robot.id = goal_pose.robot_id;
  vis_robot.x = goal_pose.pose.x;
  vis_robot.y = goal_pose.pose.y;
  vis_robot.theta = goal_pose.pose.theta;
  vis_objects_goal_->robots.push_back(vis_robot);

  vis_robot.fill_color.alpha = 0.4;
  vis_robot.caption = "final" + std::to_string(final_goal_pose.robot_id);
  vis_robot.id = final_goal_pose.robot_id;
  vis_robot.x = final_goal_pose.pose.x;
  vis_robot.y = final_goal_pose.pose.y;
  vis_robot.theta = final_goal_pose.pose.theta;
  vis_objects_goal_->robots.push_back(vis_robot);

  vis_robot.fill_color.alpha = 0.4;
  vis_robot.caption = "virtual_target" + std::to_string(final_goal_pose.robot_id);
  vis_robot.id = final_goal_pose.robot_id;
  vis_robot.x = virtual_target_pose.pose.x;
  vis_robot.y = virtual_target_pose.pose.y;
  vis_robot.theta = virtual_target_pose.pose.theta;
  vis_objects_goal_->robots.push_back(vis_robot);

  // ロボットから目標位置までの直線を引く
  VisLine vis_line;
  vis_line.p1.x = robot.pos.x;
  vis_line.p1.y = robot.pos.y;
  vis_line.p2.x = goal_pose.pose.x;
  vis_line.p2.y = goal_pose.pose.y;
  vis_line.size = 1;
  vis_line.color.name = "violet";
  vis_line.caption = "path" + std::to_string(goal_pose.robot_id);
  vis_objects_goal_->lines.push_back(vis_line);

  // 目標位置から最終目標位置までの直線を引く
  vis_line.p1.x = final_goal_pose.pose.x;
  vis_line.p1.y = final_goal_pose.pose.y;
  vis_objects_goal_->lines.push_back(vis_line);

  return true;
}

void VisualizationDataHandler::publish_and_reset_vis_goal(void)
{
  if (!vis_objects_goal_) {
    return;
  }
  if (vis_objects_goal_->robots.size() <= 0) {
    return;
  }

  vis_objects_goal_->layer = "controller";
  vis_objects_goal_->sub_layer = "goal";
  vis_objects_goal_->z_order = 2;

  pub_vis_objects_->publish(std::move(vis_objects_goal_));
  vis_objects_goal_ = std::make_unique<VisualizerObjects>();
}


}  // namespace consai_robot_controller
