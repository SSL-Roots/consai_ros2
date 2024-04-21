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

#ifndef CONSAI_ROBOT_CONTROLLER__TACTIC__TACTIC_OBSTACLE_AVOIDANCE_HPP_
#define CONSAI_ROBOT_CONTROLLER__TACTIC__TACTIC_OBSTACLE_AVOIDANCE_HPP_

#include <memory>

#include "consai_msgs/msg/state2_d.hpp"
#include "consai_robot_controller/detection_extractor.hpp"
#include "robocup_ssl_msgs/msg/tracked_ball.hpp"
#include "robocup_ssl_msgs/msg/tracked_frame.hpp"
#include "robocup_ssl_msgs/msg/tracked_robot.hpp"


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
  bool avoid_placement_area(
    const TrackedRobot & my_robot, const State & goal_pose, const TrackedBall & ball,
    const State & designated_position, State & avoidance_pose) const;
  bool avoid_pushing_robots(
    const TrackedRobot & my_robot, const State & goal_pose,
    State & avoidance_pose) const;
  bool avoid_ball_500mm(
    const TrackedRobot & my_robot,
    const State & final_goal_pose,
    const State & goal_pose, const TrackedBall & ball,
    State & avoidance_pose) const;
  bool avoid_defense_area(
    const TrackedRobot & my_robot, const State & goal_pose,
    State & avoidance_pose) const;

private:
  std::shared_ptr<DetectionExtractor> detection_;

  const double field_half_length_ = 6.0;
  const double field_half_width_ = 4.5;
  const double field_boundary_width_ = 0.3;
};

}  // namespace tactic

#endif  // CONSAI_ROBOT_CONTROLLER__TACTIC__TACTIC_OBSTACLE_AVOIDANCE_HPP_
