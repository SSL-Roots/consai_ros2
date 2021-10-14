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
#include "rclcpp/rclcpp.hpp"
#include "robocup_ssl_msgs/msg/robot_id.hpp"

using namespace std::chrono_literals;

namespace consai_vision_tracker
{

using std::placeholders::_1;

Tracker::Tracker(const rclcpp::NodeOptions & options)
: Node("tracker", options)
{
  const auto UPDATE_RATE = 0.01s;

  ball_tracker_ = std::make_shared<BallTracker>(UPDATE_RATE.count());
  for(int i=0; i<16; i++){
    blue_robot_tracker_.push_back(std::make_shared<RobotTracker>(RobotId::TEAM_COLOR_BLUE, i));
    yellow_robot_tracker_.push_back(std::make_shared<RobotTracker>(RobotId::TEAM_COLOR_YELLOW, i));
  }

  timer_ = create_wall_timer(UPDATE_RATE, std::bind(&Tracker::on_timer, this));

  pub_tracked_ = create_publisher<TrackedFrame>("detection_tracked", 10);
  sub_detection_ = create_subscription<DetectionFrame>(
    "detection", 10, std::bind(&Tracker::callback_detection, this, _1));
}

void Tracker::on_timer()
{
  const double DURATION_TIME = 0.01;
  auto tracked_msg = std::make_unique<TrackedFrame>();

  tracked_msg->balls.push_back(ball_tracker_->update());

  for(auto tracker : blue_robot_tracker_){
    tracked_msg->robots.push_back(tracker->update(DURATION_TIME));
  }

  for(auto tracker : yellow_robot_tracker_){
    tracked_msg->robots.push_back(tracker->update(DURATION_TIME));
  }

  pub_tracked_->publish(std::move(tracked_msg));
}

void Tracker::callback_detection(const DetectionFrame::SharedPtr msg)
{
  for(auto ball : msg->balls){
    ball_tracker_->push_back_observation(ball);
  }

  for(auto blue_robot : msg->robots_blue){
    if(blue_robot.robot_id.size() > 0){
      blue_robot_tracker_[blue_robot.robot_id[0]]->push_back_observation(blue_robot);
    }
  }

  for(auto yellow_robot: msg->robots_yellow){
    if(yellow_robot.robot_id.size() > 0){
      yellow_robot_tracker_[yellow_robot.robot_id[0]]->push_back_observation(yellow_robot);
    }
  }
}

}  // namespace consai_vision_tracker

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(consai_vision_tracker::Tracker)
