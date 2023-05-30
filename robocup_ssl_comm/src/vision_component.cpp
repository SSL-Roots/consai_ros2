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

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robocup_ssl_comm/vision_component.hpp"

using namespace std::chrono_literals;

namespace robocup_ssl_comm
{

Vision::Vision(const rclcpp::NodeOptions & options)
: Node("vision", options)
{
  declare_parameter("multicast_address", "224.5.23.2");
  declare_parameter("multicast_port", 10006);
  receiver_ = std::make_unique<multicast::MulticastReceiver>(
    get_parameter("multicast_address").get_value<std::string>(), get_parameter(
      "multicast_port").get_value<int>());
  pub_detection_ = create_publisher<robocup_ssl_msgs::msg::DetectionFrame>("detection", 10);
  pub_geometry_ = create_publisher<robocup_ssl_msgs::msg::GeometryData>("geometry", 10);

  timer_ = create_wall_timer(10ms, std::bind(&Vision::on_timer, this));
}

void Vision::on_timer()
{
  while (receiver_->available()) {
    std::vector<char> buf(2048);
    const size_t size = receiver_->receive(buf);

    if (size > 0) {
      SSL_WrapperPacket packet;
      packet.ParseFromString(std::string(buf.begin(), buf.end()));

      if (packet.has_detection()) {
        publish_detection(packet.detection());
      }

      if (packet.has_geometry()) {
        publish_geometry(packet.geometry());
      }
    }
  }
}

void Vision::publish_detection(const SSL_DetectionFrame & detection_frame)
{
  auto detection_msg = std::make_unique<robocup_ssl_msgs::msg::DetectionFrame>();

  detection_msg->frame_number = detection_frame.frame_number();
  detection_msg->t_capture = detection_frame.t_capture();
  detection_msg->t_sent = detection_frame.t_sent();
  detection_msg->camera_id = detection_frame.camera_id();

  for (const auto & ball : detection_frame.balls()) {
    robocup_ssl_msgs::msg::DetectionBall detection_ball;
    detection_ball.confidence = ball.confidence();
    if (ball.has_area()) {detection_ball.area.push_back(ball.area());}
    detection_ball.x = ball.x();
    detection_ball.y = ball.y();
    if (ball.has_z()) {detection_ball.z.push_back(ball.z());}
    detection_ball.pixel_x = ball.pixel_x();
    detection_ball.pixel_y = ball.pixel_y();

    detection_msg->balls.push_back(detection_ball);
  }

  for (const auto & robot : detection_frame.robots_yellow()) {
    robocup_ssl_msgs::msg::DetectionRobot msg_robot;
    msg_robot.confidence = robot.confidence();
    if (robot.has_robot_id()) {msg_robot.robot_id.push_back(robot.robot_id());}
    msg_robot.x = robot.x();
    msg_robot.y = robot.y();
    if (robot.has_orientation()) {msg_robot.orientation.push_back(robot.orientation());}
    msg_robot.pixel_x = robot.pixel_x();
    msg_robot.pixel_y = robot.pixel_y();
    if (robot.has_height()) {msg_robot.height.push_back(robot.height());}

    detection_msg->robots_yellow.push_back(msg_robot);
  }

  for (const auto & robot : detection_frame.robots_blue()) {
    robocup_ssl_msgs::msg::DetectionRobot msg_robot;
    msg_robot.confidence = robot.confidence();
    if (robot.has_robot_id()) {msg_robot.robot_id.push_back(robot.robot_id());}
    msg_robot.x = robot.x();
    msg_robot.y = robot.y();
    if (robot.has_orientation()) {msg_robot.orientation.push_back(robot.orientation());}
    msg_robot.pixel_x = robot.pixel_x();
    msg_robot.pixel_y = robot.pixel_y();
    if (robot.has_height()) {msg_robot.height.push_back(robot.height());}

    detection_msg->robots_blue.push_back(msg_robot);
  }
  pub_detection_->publish(std::move(detection_msg));
}

void Vision::publish_geometry(const SSL_GeometryData & geometry_data)
{
  auto geometry_msg = std::make_unique<robocup_ssl_msgs::msg::GeometryData>();
  set_geometry_field_size(geometry_msg->field, geometry_data.field());
  for (const auto & data_calib : geometry_data.calib()) {
    geometry_msg->calib.push_back(parse_calib(data_calib));
  }

  pub_geometry_->publish(std::move(geometry_msg));
}

void Vision::set_geometry_field_size(
  robocup_ssl_msgs::msg::GeometryFieldSize & msg_field,
  const SSL_GeometryFieldSize & data_field)
{
  msg_field.field_length = data_field.field_length();
  msg_field.field_width = data_field.field_width();
  msg_field.goal_width = data_field.goal_width();
  msg_field.goal_depth = data_field.goal_depth();
  msg_field.boundary_width = data_field.boundary_width();
  for (const auto & line : data_field.field_lines()) {
    robocup_ssl_msgs::msg::FieldLineSegment msg_line;
    msg_line.name = line.name();
    msg_line.p1.x = line.p1().x();
    msg_line.p1.y = line.p1().y();
    msg_line.p2.x = line.p2().x();
    msg_line.p2.y = line.p2().y();
    msg_line.thickness = line.thickness();
    if (line.has_type()) {msg_line.type.push_back(line.type());}

    msg_field.field_lines.push_back(msg_line);
  }
  for (const auto & arc : data_field.field_arcs()) {
    robocup_ssl_msgs::msg::FieldCircularArc msg_arc;
    msg_arc.name = arc.name();
    msg_arc.center.x = arc.center().x();
    msg_arc.center.y = arc.center().y();
    msg_arc.radius = arc.radius();
    msg_arc.a1 = arc.a1();
    msg_arc.a2 = arc.a2();
    msg_arc.thickness = arc.thickness();
    if (arc.has_type()) {msg_arc.type.push_back(arc.type());}

    msg_field.field_arcs.push_back(msg_arc);
  }
  if (data_field.has_penalty_area_depth()) {
    msg_field.penalty_area_depth.push_back(data_field.penalty_area_depth());
  }
  if (data_field.has_penalty_area_width()) {
    msg_field.penalty_area_width.push_back(data_field.penalty_area_width());
  }
}

robocup_ssl_msgs::msg::GeometryCameraCalibration Vision::parse_calib(
  const SSL_GeometryCameraCalibration & data_calib)
{
  robocup_ssl_msgs::msg::GeometryCameraCalibration msg_calib;
  msg_calib.camera_id = data_calib.camera_id();
  msg_calib.focal_length = data_calib.focal_length();
  msg_calib.principal_point_x = data_calib.principal_point_x();
  msg_calib.principal_point_y = data_calib.principal_point_y();
  msg_calib.distortion = data_calib.distortion();
  msg_calib.q0 = data_calib.q0();
  msg_calib.q1 = data_calib.q1();
  msg_calib.q2 = data_calib.q2();
  msg_calib.q3 = data_calib.q3();
  msg_calib.tx = data_calib.tx();
  msg_calib.ty = data_calib.ty();
  msg_calib.tz = data_calib.tz();
  if (data_calib.has_derived_camera_world_tx()) {
    msg_calib.derived_camera_world_tx.push_back(data_calib.derived_camera_world_tx());
  }
  if (data_calib.has_derived_camera_world_ty()) {
    msg_calib.derived_camera_world_ty.push_back(data_calib.derived_camera_world_ty());
  }
  if (data_calib.has_derived_camera_world_tz()) {
    msg_calib.derived_camera_world_tz.push_back(data_calib.derived_camera_world_tz());
  }
  if (data_calib.has_pixel_image_width()) {
    msg_calib.pixel_image_width.push_back(data_calib.pixel_image_width());
  }
  if (data_calib.has_pixel_image_height()) {
    msg_calib.pixel_image_height.push_back(data_calib.pixel_image_height());
  }

  return msg_calib;
}

}  // namespace robocup_ssl_comm

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(robocup_ssl_comm::Vision)
