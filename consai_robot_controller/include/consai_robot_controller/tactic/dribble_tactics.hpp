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

#ifndef CONSAI_ROBOT_CONTROLLER__TACTIC__DRIBBLE_TACTICS_HPP_
#define CONSAI_ROBOT_CONTROLLER__TACTIC__DRIBBLE_TACTICS_HPP_

#include <chrono>
#include <functional>
#include <map>
#include <string>

#include "consai_msgs/msg/state2_d.hpp"
#include "consai_robot_controller/tactic/tactic_data_set.hpp"
#include "robocup_ssl_msgs/msg/tracked_ball.hpp"
#include "robocup_ssl_msgs/msg/tracked_robot.hpp"

namespace dribble_tactics
{

using Time = std::chrono::system_clock::time_point;
using State = consai_msgs::msg::State2D;
using TrackedBall = robocup_ssl_msgs::msg::TrackedBall;
using TrackedRobot = robocup_ssl_msgs::msg::TrackedRobot;
using RobotID = unsigned int;
using TacticName = std::string;
using TacticDataSet = tactics::TacticDataSet;

class DribbleTactics
{
public:
  DribbleTactics();
  ~DribbleTactics() = default;
  bool update(
    const State & dribble_target, const TrackedRobot & my_robot, const TrackedBall & ball,
    State & parsed_pose, double & parsed_dribble_power);

private:
  std::map<RobotID, TacticName> tactic_name_;
  std::map<RobotID, Time> tactic_time_;
  std::map<TacticName, std::function<TacticName(TacticDataSet & data_set)>> tactic_functions_;
};

}  // namespace dribble_tactics

#endif  // CONSAI_ROBOT_CONTROLLER__TACTIC__DRIBBLE_TACTICS_HPP_
