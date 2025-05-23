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
#include <optional>

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
  void set_field_size(
    const double field_length, const double field_width,
    const double penalty_depth, const double penalty_width,
    const double goal_width, const double goal_depth
  );
  bool avoid_obstacles(
    const TrackedRobot & my_robot, const State & goal_pose, const std::optional<TrackedBall> & ball,
    const bool & avoid_our_robots,
    const bool & avoid_their_robots,
    const bool & avoid_ball,
    State & avoidance_pose) const;
  State avoid_obstacles(
    const TrackedRobot & my_robot,
    const State & goal_pose,
    const std::optional<TrackedBall> & ball,
    const bool & avoid_our_robots,
    const bool & avoid_their_robots,
    const bool & avoid_ball) const;
  bool avoid_placement_area(
    const TrackedRobot & my_robot, const State & goal_pose, const TrackedBall & ball,
    const State & designated_position, State & avoidance_pose) const;
  State avoid_placement_area(
    const TrackedRobot & my_robot,
    const State & goal_pose,
    const std::optional<TrackedBall> & ball,
    const State & designated_position) const;
  bool avoid_pushing_robots(
    const TrackedRobot & my_robot, const State & goal_pose,
    State & avoidance_pose) const;
  State avoid_pushing_robots(
    const TrackedRobot & my_robot, const State & goal_pose) const;
  bool avoid_ball_500mm(
    const TrackedRobot & my_robot,
    const State & final_goal_pose,
    const State & goal_pose, const TrackedBall & ball,
    State & avoidance_pose) const;
  State avoid_ball_around(
    const TrackedRobot & my_robot,
    const State & goal_pose,
    const std::optional<TrackedBall> & ball,
    const double & around_radius) const;
  bool avoid_defense_area(
    const TrackedRobot & my_robot, const State & goal_pose,
    State & avoidance_pose) const;
  State avoid_defense_area(
    const TrackedRobot & my_robot, const State & goal_pose) const;
  State avoid_goal(
    const TrackedRobot & my_robot, const State & goal_pose) const;

private:
  bool avoid_ball_around_impl(
    const TrackedRobot & my_robot,
    const State & final_goal_pose,
    const State & goal_pose,
    const TrackedBall & ball,
    const double & radius_threshold,
    State & avoidance_pose) const;

  std::shared_ptr<DetectionExtractor> detection_;

  double field_half_length_ = 6.0;
  double field_half_width_ = 4.5;
  double field_boundary_width_ = 0.3;
  double field_penalty_depth_ = 1.8;
  double field_penalty_width_ = 3.6;
  double field_goal_width_ = 1.8;
  double field_goal_depth_ = 0.18;
};

}  // namespace tactic

#endif  // CONSAI_ROBOT_CONTROLLER__TACTIC__TACTIC_OBSTACLE_AVOIDANCE_HPP_
