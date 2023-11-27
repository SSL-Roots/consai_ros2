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
#include "consai_visualizer_msgs/msg/shape_arc.hpp"
#include "consai_visualizer_msgs/msg/shape_circle.hpp"
#include "consai_visualizer_msgs/msg/shape_line.hpp"
#include "consai_visualizer_msgs/msg/shape_point.hpp"
#include "consai_visualizer_msgs/msg/shape_rectangle.hpp"
#include "consai_visualizer_msgs/msg/shape_robot.hpp"
#include "robocup_ssl_msgs/msg/robot_id.hpp"

namespace consai_robot_controller
{

using VisColor = consai_visualizer_msgs::msg::Color;
using VisArc = consai_visualizer_msgs::msg::ShapeArc;
using VisCircle = consai_visualizer_msgs::msg::ShapeCircle;
using VisLine = consai_visualizer_msgs::msg::ShapeLine;
using VisPoint = consai_visualizer_msgs::msg::ShapePoint;
using VisRect = consai_visualizer_msgs::msg::ShapeRectangle;
using VisRobot = consai_visualizer_msgs::msg::ShapeRobot;
using RobotId = robocup_ssl_msgs::msg::RobotId;

VisualizationDataHandler::VisualizationDataHandler(
  const rclcpp::Publisher<VisualizerObjects>::SharedPtr ptr)
  : pub_vis_objects_(ptr)
{
}

void VisualizationDataHandler::publish_vis_goal(
  const GoalPoses::SharedPtr goal_poses, const GoalPoses::SharedPtr final_goal_poses)
{
  // goal_posesを描画情報に変換してpublishする
  auto vis_objects = std::make_unique<VisualizerObjects>();

  vis_objects->layer = "controller";
  vis_objects->sub_layer = "goal";
  vis_objects->z_order = 2;

  for (const auto & goal_pose : goal_poses->poses) {
    VisRobot vis_robot;
    vis_robot.line_color.name = "black";
    vis_robot.fill_color.name = "violet";
    vis_robot.fill_color.alpha = 0.5;
    vis_robot.line_size = 1;
    vis_robot.caption = "goal" + std::to_string(goal_pose.robot_id);
    vis_robot.id = goal_pose.robot_id;
    vis_robot.x = goal_pose.pose.x;
    vis_robot.y = goal_pose.pose.y;
    vis_robot.theta = goal_pose.pose.theta;
    vis_objects->robots.push_back(vis_robot);
  }

  for (const auto & goal_pose : final_goal_poses->poses) {
    VisRobot vis_robot;
    vis_robot.line_color.name = "black";
    vis_robot.fill_color.name = "violet";
    vis_robot.fill_color.alpha = 0.8;
    vis_robot.line_size = 1;
    vis_robot.caption = "final" + std::to_string(goal_pose.robot_id);
    vis_robot.id = goal_pose.robot_id;
    vis_robot.x = goal_pose.pose.x;
    vis_robot.y = goal_pose.pose.y;
    vis_robot.theta = goal_pose.pose.theta;
    vis_objects->robots.push_back(vis_robot);
  }

  pub_vis_objects_->publish(std::move(vis_objects));
}


}  // namespace consai_robot_controller
