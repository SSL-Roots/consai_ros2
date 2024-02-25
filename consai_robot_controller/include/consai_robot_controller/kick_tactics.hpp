// Copyright 2023 Roots
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

#ifndef CONSAI_ROBOT_CONTROLLER__KICK_TACTICS_HPP_
#define CONSAI_ROBOT_CONTROLLER__KICK_TACTICS_HPP_

#include <chrono>
#include <map>

#include "consai_msgs/msg/state2_d.hpp"
#include "robocup_ssl_msgs/msg/tracked_ball.hpp"
#include "robocup_ssl_msgs/msg/tracked_robot.hpp"

namespace kick_tactics
{

using Time = std::chrono::system_clock::time_point;
using State = consai_msgs::msg::State2D;
using TrackedBall = robocup_ssl_msgs::msg::TrackedBall;
using TrackedRobot = robocup_ssl_msgs::msg::TrackedRobot;

enum class TacticState
{
  STOP_THE_BALL,
  BEHIND_THE_BALL,
  APPROACH_TO_BALL,
  SHOOT,
  FINISH,
};

class DataSet
{
public:
  DataSet(
    const State & parsed_pose, const State & kick_target,
    const TrackedRobot & my_robot, const TrackedBall & ball)
  : parsed_pose_(parsed_pose), kick_target_(kick_target), my_robot_(my_robot), ball_(ball)
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
  State get_parsed_pose(void) {return parsed_pose_;}
  double get_parsed_kick_power(void) {return parsed_kick_power_;}
  double get_parsed_dribble_power(void) {return parsed_dribble_power_;}
  State get_kick_target(void) {return kick_target_;}
  TrackedRobot get_my_robot(void) {return my_robot_;}
  TrackedBall get_ball(void) {return ball_;}

private:
  State parsed_pose_;
  double parsed_kick_power_;
  double parsed_dribble_power_;
  State kick_target_;
  TrackedRobot my_robot_;
  TrackedBall ball_;
};

class KickTactics
{
public:
  KickTactics();
  ~KickTactics() = default;
  bool update(
    const State & kick_target, const TrackedRobot & my_robot, const TrackedBall & ball,
    State & parsed_pose, double & parsed_kick_power, double & parsed_dribble_power);

private:
  bool need_reset_state(const TrackedRobot & my_robot, const TrackedBall & ball) const;
  bool is_same(
    const State & target, const State & current, const double threshold_distance = 0.05) const;
  bool ball_is_moving(const TrackedBall & ball) const;

  TacticState tactic_stop_the_ball(DataSet & data_set) const;
  TacticState tactic_behind_the_ball(DataSet & data_set) const;
  TacticState tactic_approach_to_ball(DataSet & data_set) const;
  TacticState tactic_shoot(DataSet & data_set) const;
  TacticState tactic_finish(DataSet & data_set) const;

  std::map<unsigned int, TacticState> tactic_state_;
  std::map<unsigned int, Time> tactic_time_;
  std::map<TacticState, std::function<TacticState(DataSet & data_set)>> tactic_functions_;
};

}  // namespace kick_tactics

#endif  // CONSAI_ROBOT_CONTROLLER__KICK_TACTICS_HPP_
