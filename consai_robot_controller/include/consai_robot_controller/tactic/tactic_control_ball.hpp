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

#ifndef CONSAI_ROBOT_CONTROLLER__TACTIC__TACTIC_CONTROL_BALL_HPP_
#define CONSAI_ROBOT_CONTROLLER__TACTIC__TACTIC_CONTROL_BALL_HPP_

#include "consai_msgs/msg/state2_d.hpp"
#include "robocup_ssl_msgs/msg/tracked_ball.hpp"
#include "robocup_ssl_msgs/msg/tracked_frame.hpp"
#include "robocup_ssl_msgs/msg/tracked_robot.hpp"

namespace tactic
{

using State = consai_msgs::msg::State2D;
using TrackedBall = robocup_ssl_msgs::msg::TrackedBall;
using TrackedFrame = robocup_ssl_msgs::msg::TrackedFrame;
using TrackedRobot = robocup_ssl_msgs::msg::TrackedRobot;

class ControlBall
{
public:
  ControlBall() = default;

  bool control_ball(
    const State & target, const TrackedRobot & my_robot, const TrackedBall & ball,
    const double & dribble_distance, State & parsed_pose, bool & need_kick,
    bool & need_dribble) const;
  bool control_ball_at_setplay(
    const State & target, const TrackedRobot & my_robot, const TrackedBall & ball,
    State & parsed_pose, bool & need_kick, bool & need_dribble) const;
  bool receive_ball(
    const TrackedRobot & my_robot, const TrackedBall & ball,
    State & parsed_pose, double & parsed_dribble_power) const;
  bool reflect_kick(
    const State & target, const TrackedRobot & my_robot, const TrackedBall & ball,
    const bool & kick_pass, State & parsed_pose, double & parsed_kick_power,
    double & parsed_dribble_power) const;

private:
  double max_shoot_speed_ = 5.0;  // m/s
  double max_pass_speed_ = 4.0;  // m/s
};

}  // namespace tactic

#endif  // CONSAI_ROBOT_CONTROLLER__TACTIC__TACTIC_CONTROL_BALL_HPP_
