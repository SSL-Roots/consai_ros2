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

#include "consai_vision_tracker/visualization_data_handler.hpp"
#include "consai_visualizer_msgs/msg/shape_arc.hpp"
#include "consai_visualizer_msgs/msg/shape_circle.hpp"
#include "consai_visualizer_msgs/msg/shape_line.hpp"
#include "consai_visualizer_msgs/msg/shape_point.hpp"
#include "consai_visualizer_msgs/msg/shape_rectangle.hpp"
#include "consai_visualizer_msgs/msg/shape_robot.hpp"
#include "robocup_ssl_msgs/msg/robot_id.hpp"

namespace consai_vision_tracker
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

void VisualizationDataHandler::publish_vis_detection(const DetectionFrame::SharedPtr msg)
{
  // detectionを描画情報に変換してpublishする
  auto vis_objects = std::make_unique<VisualizerObjects>();
  bool has_object = false;
  const auto cam_id = std::to_string(msg->camera_id);

  vis_objects->layer = "vision";
  vis_objects->sub_layer = "detection_cam" + cam_id;
  vis_objects->z_order = 1;

  VisCircle vis_ball;
  vis_ball.line_color.name = "black";
  vis_ball.fill_color.name = "orange";
  vis_ball.fill_color.alpha = 0.7;
  vis_ball.line_size = 1;
  vis_ball.radius = 0.0215;
  vis_ball.caption = cam_id;

  for (const auto & ball : msg->balls) {
    vis_ball.center.x = ball.x * 0.001;
    vis_ball.center.y = ball.y * 0.001;
    vis_objects->circles.push_back(vis_ball);
    has_object = true;
  }

  VisRobot vis_robot;
  vis_robot.line_color.name = "black";
  vis_robot.fill_color.name = "dodgerblue";
  vis_robot.fill_color.alpha = 0.7;
  vis_robot.line_size = 1;
  vis_robot.caption = cam_id;
  for (const auto & robot : msg->robots_blue) {
    if (robot.robot_id.size() <= 0) {
      continue;
    }
    vis_robot.id = robot.robot_id[0];
    vis_robot.x = robot.x * 0.001;
    vis_robot.y = robot.y * 0.001;
    if (robot.orientation.size() > 0) {
      vis_robot.theta = robot.orientation[0];
    }
    vis_objects->robots.push_back(vis_robot);
    has_object = true;
  }

  vis_robot.line_color.name = "black";
  vis_robot.fill_color.name = "yellow";
  for (const auto & robot : msg->robots_yellow) {
    if (robot.robot_id.size() <= 0) {
      continue;
    }
    vis_robot.id = robot.robot_id[0];
    vis_robot.x = robot.x * 0.001;
    vis_robot.y = robot.y * 0.001;
    if (robot.orientation.size() > 0) {
      vis_robot.theta = robot.orientation[0];
    }
    vis_objects->robots.push_back(vis_robot);
    has_object = true;
  }

  if (has_object) {
    pub_vis_objects_->publish(std::move(vis_objects));
  }
}

void VisualizationDataHandler::publish_vis_geometry(const GeometryData::SharedPtr msg)
{
  // geometryを描画情報に変換してpublishする
  auto vis_objects = std::make_unique<VisualizerObjects>();

  vis_objects->layer = "vision";
  vis_objects->sub_layer = "geometry";
  vis_objects->z_order = 0;

  for (const auto & field_line : msg->field.field_lines) {
    VisLine line;

    line.color.name = "white";
    line.size = 2;
    // 単位を[m]に変換
    line.p1.x = field_line.p1.x * 0.001;
    line.p1.y = field_line.p1.y * 0.001;
    line.p2.x = field_line.p2.x * 0.001;
    line.p2.y = field_line.p2.y * 0.001;
    line.caption = field_line.name;

    vis_objects->lines.push_back(line);
  }

  for (const auto & field_arc : msg->field.field_arcs) {
    VisArc arc;

    arc.color.name = "white";
    arc.size = 2;
    // 単位を[m]に変換
    arc.center.x = field_arc.center.x * 0.001;
    arc.center.y = field_arc.center.y * 0.001;
    arc.radius = field_arc.radius * 0.001;
    arc.start_angle = field_arc.a1;
    arc.end_angle = field_arc.a2;
    arc.caption = field_arc.name;

    vis_objects->arcs.push_back(arc);
  }

  // ペナルティマーク
  // Ref: https://robocup-ssl.github.io/ssl-rules/sslrules.html#_penalty_mark
  VisPoint point;
  point.color.name = "white";
  point.size = 6;
  point.x = -msg->field.field_length * 0.001 / 2.0 + 8.0;
  point.y = 0.0;
  point.caption = "penalty_mark_positive";
  vis_objects->points.push_back(point);

  point.x = -point.x;
  point.caption = "penalty_mark_negative";
  vis_objects->points.push_back(point);

  // フィールドの枠
  VisRect rect;
  rect.line_color.name = "black";
  rect.fill_color.alpha = 0.0;
  rect.line_size = 3;
  rect.center.x = 0.0;
  rect.center.y = 0.0;
  rect.width = (msg->field.field_length + msg->field.boundary_width * 2) * 0.001;
  rect.height = (msg->field.field_width + msg->field.boundary_width * 2) * 0.001;
  rect.caption = "wall";
  vis_objects->rects.push_back(rect);

  pub_vis_objects_->publish(std::move(vis_objects));
}

TrackedFrame::UniquePtr VisualizationDataHandler::publish_vis_tracked(
  TrackedFrame::UniquePtr msg)
{
  // tracked_frameを描画情報に変換してpublishする
  auto vis_objects = std::make_unique<VisualizerObjects>();
  vis_objects->layer = "vision";
  vis_objects->sub_layer = "tracked";
  vis_objects->z_order = 10;  // 一番上に描画する

  VisCircle vis_ball;
  vis_ball.line_color.name = "black";
  vis_ball.fill_color.name = "orange";
  vis_ball.line_size = 1;
  vis_ball.radius = 0.0215;
  for (const auto & ball : msg->balls) {
    if (ball.visibility.size() <= 0 || ball.visibility[0] < 0.5) {
      continue;
    }
    vis_ball.center.x = ball.pos.x;
    vis_ball.center.y = ball.pos.y;
    vis_objects->circles.push_back(vis_ball);

    // ボールは小さいのでボールの周りを大きな円で囲う
    vis_ball.line_color.name = "crimson";
    vis_ball.fill_color.alpha = 0.0;
    vis_ball.line_size = 2;
    vis_ball.radius = 0.8;
    vis_ball.caption = "ball is here";
    vis_objects->circles.push_back(vis_ball);

    // 速度を描画
    if (ball.vel.size() > 0) {
      const double vel_norm = std::hypot(ball.vel[0].x, ball.vel[0].y);
      VisLine ball_vel;
      // 速度の大きさに合わせて色の透明度を変える
      ball_vel.color.name = "gold";
      ball_vel.color.alpha = std::clamp(vel_norm, 0.0, 1.0);
      ball_vel.size = 2;
      ball_vel.p1.x = ball.pos.x;
      ball_vel.p1.y = ball.pos.y;
      ball_vel.p2.x = ball.pos.x + ball.vel[0].x;
      ball_vel.p2.y = ball.pos.y + ball.vel[0].y;
      ball_vel.caption = std::to_string(vel_norm);
      vis_objects->lines.push_back(ball_vel);
    }
  }

  VisRobot vis_robot;
  vis_robot.line_color.name = "black";
  vis_robot.line_size = 1;
  for (const auto & robot : msg->robots) {
    if (robot.visibility.size() <= 0 || robot.visibility[0] < 0.5) {
      continue;
    }

    vis_robot.id = robot.robot_id.id;
    if (robot.robot_id.team_color == RobotId::TEAM_COLOR_BLUE) {
      vis_robot.fill_color.name = "dodgerblue";
    } else {
      vis_robot.fill_color.name = "yellow";
    }

    vis_robot.x = robot.pos.x;
    vis_robot.y = robot.pos.y;
    vis_robot.theta = robot.orientation;
    vis_objects->robots.push_back(vis_robot);

    // 速度を描画
    if (robot.vel.size() > 0 && robot.vel_angular.size() > 0) {
      const double vel_norm = std::hypot(robot.vel[0].x, robot.vel[0].y);
      VisLine robot_vel;
      // 速度の大きさに合わせて色の透明度を変える
      // 直進速度
      robot_vel.color.name = "gold";
      robot_vel.color.alpha = std::clamp(vel_norm, 0.0, 1.0);
      robot_vel.size = 2;
      robot_vel.p1.x = robot.pos.x;
      robot_vel.p1.y = robot.pos.y;
      robot_vel.p2.x = robot.pos.x + robot.vel[0].x;
      robot_vel.p2.y = robot.pos.y + robot.vel[0].y;
      robot_vel.caption = std::to_string(vel_norm);
      vis_objects->lines.push_back(robot_vel);

      // 角速度
      const double vel_angular_norm = std::fabs(robot.vel_angular[0]);
      robot_vel.color.name = "crimson";
      robot_vel.color.alpha = std::clamp(vel_angular_norm, 0.0, 1.0);
      robot_vel.p1.x = robot.pos.x;
      robot_vel.p1.y = robot.pos.y;
      robot_vel.p2.x = robot.pos.x + robot.vel_angular[0];
      robot_vel.p2.y = robot.pos.y;
      robot_vel.caption = std::to_string(vel_angular_norm);
      vis_objects->lines.push_back(robot_vel);
    }
  }

  pub_vis_objects_->publish(std::move(vis_objects));

  return msg;
}

}  // namespace consai_vision_tracker
