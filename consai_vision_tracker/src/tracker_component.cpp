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
  estimator_ball_ = std::make_shared<Estimator>();
  for(int i=0; i<16; i++){
    estimators_blue_robot_.push_back(std::make_shared<Estimator>(RobotId::TEAM_COLOR_BLUE, i));
    estimators_yellow_robot_.push_back(std::make_shared<Estimator>(RobotId::TEAM_COLOR_YELLOW, i));
  }

  timer_ = create_wall_timer(10ms, std::bind(&Tracker::on_timer, this));

  pub_tracked_ = create_publisher<TrackedFrame>("detection_tracked", 10);
  sub_detection_ = create_subscription<DetectionFrame>(
    "detection", 10, std::bind(&Tracker::callback_detection, this, _1));
}

void Tracker::on_timer()
{
  const double DURATION_TIME = 0.016;
  auto tracked_msg = std::make_unique<TrackedFrame>();

  auto ball = estimator_ball_->estimate_ball(DURATION_TIME);
  tracked_msg->balls.push_back(ball);

  for(auto estimator : estimators_blue_robot_){
    tracked_msg->robots.push_back(estimator->estimate_robot(DURATION_TIME));
  }

  for(auto estimator : estimators_yellow_robot_){
    tracked_msg->robots.push_back(estimator->estimate_robot(DURATION_TIME));
  }

  pub_tracked_->publish(std::move(tracked_msg));
}

void Tracker::callback_detection(const DetectionFrame::SharedPtr msg)
{
  auto camera_id = msg->camera_id;
  auto t_capture = msg->t_capture;

  for(auto ball : msg->balls){
    estimator_ball_->set_observation(ball, camera_id, t_capture);
  }

  for(auto blue_robot : msg->robots_blue){
    if(blue_robot.robot_id.size() > 0){
      estimators_blue_robot_[blue_robot.robot_id[0]]->set_observation(blue_robot, camera_id, t_capture);
    }
  }

  for(auto yellow_robot: msg->robots_yellow){
    if(yellow_robot.robot_id.size() > 0){
      estimators_yellow_robot_[yellow_robot.robot_id[0]]->set_observation(yellow_robot, camera_id, t_capture);
    }
  }
}

}  // namespace consai_vision_tracker

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(consai_vision_tracker::Tracker)
