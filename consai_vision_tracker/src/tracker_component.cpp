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

using namespace std::chrono_literals;

namespace consai_vision_tracker
{

using std::placeholders::_1;

Tracker::Tracker(const rclcpp::NodeOptions & options)
: Node("tracker", options)
{
  timer_ = create_wall_timer(10ms, std::bind(&Tracker::on_timer, this));

  pub_tracked_ = create_publisher<TrackedFrame>("detection_tracked", 1);
  sub_detection_ = create_subscription<DetectionFrame>(
    "detection", 10, std::bind(&Tracker::callback_detection, this, _1));
}

void Tracker::on_timer()
{
  // RCLCPP_INFO(this->get_logger(), "Hello World!");

  // ストックされたデータを処理する
  auto tracked_msg = std::make_unique<TrackedFrame>();
  // tracked_msg->frame_number = tracked_msg.frame_number;
  pub_tracked_->publish(std::move(tracked_frame_));
}

void Tracker::callback_detection(const DetectionFrame::SharedPtr msg)
{
  tracked_frame_.frame_number = msg->frame_number;
  // std::cout<<"id, capture, sent";
  // std::cout<<msg->camera_id<<",";
  // std::cout<<std::to_string(msg->t_capture)<<",";
  // std::cout<<std::to_string(msg->t_sent)<<","<<std::endl;

  track_ball(msg->balls);
}

void Tracker::track_ball(const std::vector<DetectionBall> & balls)
{
  for(auto ball : balls){
    std::cout<<"x,y"<<std::to_string(ball.x)<<","<<std::to_string(ball.y)<<std::endl;
  }
}

}  // namespace consai_vision_tracker

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(consai_vision_tracker::Tracker)
