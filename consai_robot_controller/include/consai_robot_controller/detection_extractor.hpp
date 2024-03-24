// Copyright 2024 Roots
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

#ifndef CONSAI_ROBOT_CONTROLLER__DETECTION_EXTRACTOR_HPP_
#define CONSAI_ROBOT_CONTROLLER__DETECTION_EXTRACTOR_HPP_

#include <memory>
#include <vector>

#include "robocup_ssl_msgs/msg/tracked_ball.hpp"
#include "robocup_ssl_msgs/msg/tracked_frame.hpp"
#include "robocup_ssl_msgs/msg/tracked_robot.hpp"


namespace parser
{

using TrackedBall = robocup_ssl_msgs::msg::TrackedBall;
using TrackedFrame = robocup_ssl_msgs::msg::TrackedFrame;
using TrackedRobot = robocup_ssl_msgs::msg::TrackedRobot;

class DetectionExtractor
{
 public:
  explicit DetectionExtractor(const double visibility_threshold = 0.01);

  void set_detection_tracked(const TrackedFrame::SharedPtr detection_tracked) {
    detection_tracked_ = detection_tracked;
  }
  bool extract_robot(
    const unsigned int robot_id, const bool team_is_yellow,
    TrackedRobot & my_robot) const;
  bool extract_ball(TrackedBall & my_ball) const;
  std::vector<TrackedRobot> extract_robots() const;
  std::vector<unsigned int> active_robot_id_list(const bool team_is_yellow) const;

 private:
  double visibility_threshold_ = 0.01;
  std::shared_ptr<TrackedFrame> detection_tracked_;
};

}  // namespace parser


#endif  // CONSAI_ROBOT_CONTROLLER__DETECTION_EXTRACTOR_HPP_
