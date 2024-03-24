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

#ifndef CONSAI_ROBOT_CONTROLLER__TACTIC_OBSTACLE_AVOIDANCE_HPP_
#define CONSAI_ROBOT_CONTROLLER__TACTIC_OBSTACLE_AVOIDANCE_HPP_

#include <memory>

#include "consai_msgs/msg/state2_d.hpp"
#include "robocup_ssl_msgs/msg/tracked_ball.hpp"
#include "robocup_ssl_msgs/msg/tracked_frame.hpp"
#include "robocup_ssl_msgs/msg/tracked_robot.hpp"

#include "consai_robot_controller/detection_extractor.hpp"

namespace tactic
{

using DetectionExtractor = parser::DetectionExtractor;

using State = consai_msgs::msg::State2D;
using TrackedBall = robocup_ssl_msgs::msg::TrackedBall;
using TrackedFrame = robocup_ssl_msgs::msg::TrackedFrame;
using TrackedRobot = robocup_ssl_msgs::msg::TrackedRobot;

class ObstacleAvoidance
{
 public:
  explicit ObstacleAvoidance(const std::shared_ptr<DetectionExtractor> & detection_extractor);
  bool avoid_obstacles(
    const TrackedRobot & my_robot, const State & goal_pose, const TrackedBall & ball,
    const bool & avoid_ball, State & avoidance_pose) const;

 private:
  std::shared_ptr<DetectionExtractor> detection_;
};


}



#endif  // CONSAI_ROBOT_CONTROLLER__TACTION_OBSTACLE_AVOIDANCE_HPP_
