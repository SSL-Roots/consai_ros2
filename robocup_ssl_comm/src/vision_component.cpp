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
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robocup_ssl_comm/vision_component.hpp"

using namespace std::chrono_literals;

namespace robocup_ssl_comm
{

Vision::Vision(const rclcpp::NodeOptions & options)
: Node("vision", options)
{
  receiver_ = std::make_unique<multicast::MulticastReceiver>("224.5.23.2", 10006);
  pub_detection_ = create_publisher<robocup_ssl_msgs::msg::DetectionFrame>("detection_frame", 1);

  timer_ = create_wall_timer(10ms, std::bind(&Vision::on_timer, this));
}

void Vision::on_timer()
{
  while(receiver_->available()){
    std::vector<char> buf(2048);
    const size_t size = receiver_->receive(buf);

    if(size > 0){
      SSL_WrapperPacket packet;
      packet.ParseFromString(std::string(buf.begin(), buf.end()));

      if(packet.has_detection()){
        publish_detection(packet.detection());
      }
        
      if(packet.has_geometry()){
        RCLCPP_INFO(this->get_logger(), "Has Geometry!");
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

  for(auto ball : detection_frame.balls()){
    robocup_ssl_msgs::msg::DetectionBall detection_ball;
    detection_ball.confidence = ball.confidence();
    if(ball.has_area()) detection_ball.area = ball.area();
    detection_ball.x = ball.x();
    detection_ball.y = ball.y();
    if(ball.has_x()) detection_ball.z = ball.z();
    detection_ball.pixel_x = ball.pixel_x();
    detection_ball.pixel_y = ball.pixel_y();

    detection_msg->balls.push_back(detection_ball);
  }

  for(auto robot : detection_frame.robots_yellow()){
    robocup_ssl_msgs::msg::DetectionRobot detection_robot;
    detection_robot.confidence = robot.confidence();
    if(robot.has_robot_id()) detection_robot.robot_id = robot.robot_id();
    detection_robot.x = robot.x();
    detection_robot.y = robot.y();
    if(robot.has_orientation()) detection_robot.orientation = robot.orientation();
    detection_robot.pixel_x = robot.pixel_x();
    detection_robot.pixel_y = robot.pixel_y();
    if(robot.has_height()) detection_robot.height = robot.height();

    detection_msg->robots_yellow.push_back(detection_robot);
  }

  for(auto robot : detection_frame.robots_blue()){
    robocup_ssl_msgs::msg::DetectionRobot detection_robot;
    detection_robot.confidence = robot.confidence();
    if(robot.has_robot_id()) detection_robot.robot_id = robot.robot_id();
    detection_robot.x = robot.x();
    detection_robot.y = robot.y();
    if(robot.has_orientation()) detection_robot.orientation = robot.orientation();
    detection_robot.pixel_x = robot.pixel_x();
    detection_robot.pixel_y = robot.pixel_y();
    if(robot.has_height()) detection_robot.height = robot.height();

    detection_msg->robots_blue.push_back(detection_robot);
  }
  pub_detection_->publish(std::move(detection_msg));
}

}  // namespace robocup_ssl_comm

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(robocup_ssl_comm::Vision)
