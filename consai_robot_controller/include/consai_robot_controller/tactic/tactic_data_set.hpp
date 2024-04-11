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


#ifndef CONSAI_ROBOT_CONTROLLER__TACTIC__TACTIC_DATA_SET_HPP_
#define CONSAI_ROBOT_CONTROLLER__TACTIC__TACTIC_DATA_SET_HPP_

#include "consai_msgs/msg/state2_d.hpp"
#include "robocup_ssl_msgs/msg/tracked_ball.hpp"
#include "robocup_ssl_msgs/msg/tracked_robot.hpp"

namespace tactics
{

using State = consai_msgs::msg::State2D;
using TrackedBall = robocup_ssl_msgs::msg::TrackedBall;
using TrackedRobot = robocup_ssl_msgs::msg::TrackedRobot;

class TacticDataSet
{
public:
  TacticDataSet(
    const TrackedRobot & my_robot, const TrackedBall & ball,
    const State & target,
    const State & parsed_pose,
    const double & parsed_kick_power,
    const double & parsed_dribble_power
  )
  : my_robot_(my_robot), ball_(ball),
    target_(target),
    parsed_pose_(parsed_pose),
    parsed_kick_power_(parsed_kick_power),
    parsed_dribble_power_(parsed_dribble_power)
  {}

  void set_parsed_pose(const State & parsed_pose) {parsed_pose_ = parsed_pose;}
  void set_parsed_kick_power(const double & parsed_kick_power)
  {
    parsed_kick_power_ = parsed_kick_power;
  }
  void set_parsed_dribble_power(const double & parsed_dribble_power)
  {
    parsed_dribble_power_ = parsed_dribble_power;
  }
  void set_pass(const bool & is_pass) {is_pass_ = is_pass;}

  TrackedRobot get_my_robot(void) {return my_robot_;}
  TrackedBall get_ball(void) {return ball_;}
  State get_target(void) {return target_;}
  State get_parsed_pose(void) {return parsed_pose_;}
  double get_parsed_kick_power(void) {return parsed_kick_power_;}
  double get_parsed_dribble_power(void) {return parsed_dribble_power_;}
  bool is_pass(void) {return is_pass_;}

private:
  TrackedRobot my_robot_;
  TrackedBall ball_;
  State target_;
  State parsed_pose_;
  double parsed_kick_power_;
  double parsed_dribble_power_;
  bool is_pass_ = false;
};

}  // namespace tactics

#endif  // CONSAI_ROBOT_CONTROLLER__TACTIC__TACTIC_DATA_SET_HPP_
