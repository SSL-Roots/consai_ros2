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

#ifndef CONSAI_VISION_TRACKER__TRACKER_COMPONENT_HPP_
#define CONSAI_VISION_TRACKER__TRACKER_COMPONENT_HPP_

#include <memory>
#include <vector>

#include "consai_msgs/msg/robot_local_velocities.hpp"
#include "consai_msgs/msg/robot_local_velocity.hpp"
#include "consai_vision_tracker/visibility_control.h"
#include "consai_vision_tracker/ball_tracker.hpp"
#include "consai_vision_tracker/robot_tracker.hpp"
#include "consai_vision_tracker/visualization_data_handler.hpp"
#include "robocup_ssl_msgs/msg/detection_ball.hpp"
#include "robocup_ssl_msgs/msg/detection_frame.hpp"
#include "robocup_ssl_msgs/msg/detection_robot.hpp"
#include "robocup_ssl_msgs/msg/geometry_data.hpp"
#include "robocup_ssl_msgs/msg/tracked_frame.hpp"
#include "robocup_ssl_msgs/msg/tracked_ball.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace consai_vision_tracker
{

using RobotLocalVelocity = consai_msgs::msg::RobotLocalVelocity;
using RobotLocalVelocities = consai_msgs::msg::RobotLocalVelocities;
using DetectionBall = robocup_ssl_msgs::msg::DetectionBall;
using DetectionFrame = robocup_ssl_msgs::msg::DetectionFrame;
using DetectionRobot = robocup_ssl_msgs::msg::DetectionRobot;
using GeometryData = robocup_ssl_msgs::msg::GeometryData;
using TrackedFrame = robocup_ssl_msgs::msg::TrackedFrame;

class Tracker : public rclcpp::Node
{
public:
  CONSAI_VISION_TRACKER_PUBLIC
  explicit Tracker(const rclcpp::NodeOptions & options);

protected:
  void on_timer();

private:
  void gen_robot_trackers(const unsigned int num);
  void callback_detection(const DetectionFrame::SharedPtr msg);
  void callback_detection_invert(const DetectionFrame::SharedPtr msg);
  void callback_geometry(const GeometryData::SharedPtr msg);
  void invert_ball(DetectionBall & ball);
  void invert_robot(DetectionRobot & robot);

  std::chrono::duration<double> update_rate_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<DetectionFrame>::SharedPtr sub_detection_;
  rclcpp::Subscription<GeometryData>::SharedPtr sub_geometry_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_consai_param_rule_;
  rclcpp::Publisher<TrackedFrame>::SharedPtr pub_tracked_;
  rclcpp::Publisher<RobotLocalVelocities>::SharedPtr pub_robot_velocities_;
  std::shared_ptr<BallTracker> ball_tracker_;
  std::vector<std::shared_ptr<RobotTracker>> blue_robot_tracker_;
  std::vector<std::shared_ptr<RobotTracker>> yellow_robot_tracker_;
  std::shared_ptr<VisualizationDataHandler> vis_data_handler_;
};

}  // namespace consai_vision_tracker

#endif  // CONSAI_VISION_TRACKER__TRACKER_COMPONENT_HPP_
