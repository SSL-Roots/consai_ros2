// Copyright 2021 Roots
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

#include "consai_vision_tracker/tracker_component.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <utility>

#include "consai_visualizer_msgs/msg/color.hpp"
#include "consai_visualizer_msgs/msg/shape_arc.hpp"
#include "consai_visualizer_msgs/msg/shape_circle.hpp"
#include "consai_visualizer_msgs/msg/shape_line.hpp"
#include "consai_visualizer_msgs/msg/shape_point.hpp"
#include "consai_visualizer_msgs/msg/shape_rectangle.hpp"
#include "consai_visualizer_msgs/msg/shape_robot.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robocup_ssl_msgs/msg/robot_id.hpp"

using namespace std::chrono_literals;

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
using std::placeholders::_1;

Tracker::Tracker(const rclcpp::NodeOptions & options)
: Node("tracker", options)
{
  const auto UPDATE_RATE = 0.01s;

  ball_tracker_ = std::make_shared<BallTracker>(UPDATE_RATE.count());
  for (int i = 0; i < 16; i++) {
    blue_robot_tracker_.push_back(
      std::make_shared<RobotTracker>(
        RobotId::TEAM_COLOR_BLUE, i,
        UPDATE_RATE.count()));
    yellow_robot_tracker_.push_back(
      std::make_shared<RobotTracker>(
        RobotId::TEAM_COLOR_YELLOW, i,
        UPDATE_RATE.count()));
  }

  timer_ = create_wall_timer(UPDATE_RATE, std::bind(&Tracker::on_timer, this));

  pub_tracked_ = create_publisher<TrackedFrame>("detection_tracked", 10);
  pub_robot_velocities_ = create_publisher<RobotLocalVelocities>("robot_local_velocities", 10);
  pub_vis_objects_ = create_publisher<VisualizerObjects>(
    "visualizer_objects", rclcpp::SensorDataQoS());

  declare_parameter("invert", false);

  if (get_parameter("invert").get_value<bool>()) {
    sub_detection_ = create_subscription<DetectionFrame>(
      "detection", 10, std::bind(&Tracker::callback_detection_invert, this, _1));
  } else {
    sub_detection_ = create_subscription<DetectionFrame>(
      "detection", 10, std::bind(&Tracker::callback_detection, this, _1));
  }

  sub_geometry_ = create_subscription<GeometryData>(
    "geometry", 10, std::bind(&Tracker::callback_geometry, this, _1));
}

void Tracker::on_timer()
{
  auto tracked_msg = std::make_unique<TrackedFrame>();
  auto robot_vel_msg = std::make_unique<RobotLocalVelocities>();

  tracked_msg->balls.push_back(ball_tracker_->update());

  for (auto && tracker : blue_robot_tracker_) {
    tracked_msg->robots.push_back(tracker->update());
    robot_vel_msg->velocities.push_back(tracker->calc_local_velocity());
  }

  for (auto && tracker : yellow_robot_tracker_) {
    tracked_msg->robots.push_back(tracker->update());
    robot_vel_msg->velocities.push_back(tracker->calc_local_velocity());
  }

  pub_tracked_->publish(std::move(tracked_msg));
  pub_robot_velocities_->publish(std::move(robot_vel_msg));
}

void Tracker::callback_detection(const DetectionFrame::SharedPtr msg)
{
  for (const auto & ball : msg->balls) {
    ball_tracker_->push_back_observation(ball);
  }

  for (const auto & blue_robot : msg->robots_blue) {
    if (blue_robot.robot_id.size() > 0) {
      blue_robot_tracker_[blue_robot.robot_id[0]]->push_back_observation(blue_robot);
    }
  }

  for (const auto & yellow_robot : msg->robots_yellow) {
    if (yellow_robot.robot_id.size() > 0) {
      yellow_robot_tracker_[yellow_robot.robot_id[0]]->push_back_observation(yellow_robot);
    }
  }

  publish_vis_detection(msg);
}

void Tracker::callback_detection_invert(const DetectionFrame::SharedPtr msg)
{
  // detectionトピックの情報を上下左右反転するcallback関数

  for (auto && ball : msg->balls) {
    invert_ball(ball);
    ball_tracker_->push_back_observation(ball);
  }

  for (auto && blue_robot : msg->robots_blue) {
    if (blue_robot.robot_id.size() > 0) {
      invert_robot(blue_robot);
      blue_robot_tracker_[blue_robot.robot_id[0]]->push_back_observation(blue_robot);
    }
  }

  for (auto && yellow_robot : msg->robots_yellow) {
    if (yellow_robot.robot_id.size() > 0) {
      invert_robot(yellow_robot);
      yellow_robot_tracker_[yellow_robot.robot_id[0]]->push_back_observation(yellow_robot);
    }
  }

  publish_vis_detection(msg);
}

void Tracker::callback_geometry(const GeometryData::SharedPtr msg)
{
  publish_vis_geometry(msg);
}

void Tracker::publish_vis_detection(const DetectionFrame::SharedPtr msg)
{
  // detectionを描画情報に変換してpublishする
  auto vis_objects = std::make_unique<VisualizerObjects>();
  bool has_object = false;

  vis_objects->layer = "vision";
  vis_objects->sub_layer = "detection_cam" + std::to_string(msg->camera_id);
  vis_objects->z_order = 1;

  VisCircle vis_ball;
  vis_ball.line_color.name = "black";
  vis_ball.fill_color.name = "orange";
  vis_ball.line_size = 1;
  vis_ball.radius = 0.0215;

  for (const auto & ball : msg->balls) {
    vis_ball.center.x = ball.x * 0.001;
    vis_ball.center.y = ball.y * 0.001;
    vis_objects->circles.push_back(vis_ball);
    has_object = true;
  }

  VisRobot vis_robot;
  vis_robot.line_color.name = "black";
  vis_robot.fill_color.name = "dodgerblue";
  vis_robot.line_size = 1;
  vis_robot.caption = "detection";
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

void Tracker::publish_vis_geometry(const GeometryData::SharedPtr msg)
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

void Tracker::invert_ball(DetectionBall & ball)
{
  // detection ballを上下左右反転する
  ball.x = -ball.x;
  ball.y = -ball.y;
  ball.pixel_x = -ball.pixel_x;
  ball.pixel_y = -ball.pixel_y;
}

void Tracker::invert_robot(DetectionRobot & robot)
{
  // detection robotを上下左右反転する
  robot.x = -robot.x;
  robot.y = -robot.y;
  robot.pixel_x = -robot.pixel_x;
  robot.pixel_y = -robot.pixel_y;
  if (robot.orientation.size() > 0) {
    robot.orientation[0] -= std::copysign(M_PI, robot.orientation[0]);
  }
}

}  // namespace consai_vision_tracker

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(consai_vision_tracker::Tracker)
