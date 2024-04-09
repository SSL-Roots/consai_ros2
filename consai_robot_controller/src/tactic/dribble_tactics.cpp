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

#include "consai_robot_controller/tactic/dribble_tactics.hpp"
#include "consai_tools/geometry_tools.hpp"

#include <iostream>

namespace dribble_tactics
{

namespace tools = geometry_tools;
namespace chrono = std::chrono;

static constexpr double ROBOT_RADIUS = 0.09;  // ロボットの半径
static constexpr double DRIBBLE_CATCH = 1.0;
static constexpr double DRIBBLE_RELEASE = 0.0;
static constexpr auto APPROACH = "APPROACH";
static constexpr auto CATCH = "CATCH";
static constexpr auto CARRY = "CARRY";


DribbleTactics::DribbleTactics()
{
  tactic_functions_[APPROACH] = [this](TacticDataSet & data_set) -> TacticName {
    // 現在位置からボールに対してまっすぐ進む
    const auto robot_pose = tools::pose_state(data_set.get_my_robot());
    const auto ball_pose = tools::pose_state(data_set.get_ball());

    const tools::Trans trans_BtoR(ball_pose, tools::calc_angle(ball_pose, robot_pose));
    const auto new_pose = trans_BtoR.inverted_transform(ROBOT_RADIUS * 2.0, 0.0, -M_PI);

    data_set.set_parsed_pose(new_pose);
    data_set.set_parsed_dribble_power(DRIBBLE_RELEASE);

    if (tools::is_same(robot_pose, new_pose, 0.1, tools::to_radians(5.0))) {
      return CATCH;
    }

    std::cout << "Approaching" << std::endl;
    
    return APPROACH;
  };

  tactic_functions_[CATCH] = [this](TacticDataSet & data_set) -> TacticName {
    // ドリブラーを駆動させながら前進する
    // ボールをキャッチするため、一定時間は行動を継続する
    constexpr auto CATCHING_TIME = 1.0;  // seconds

    const auto robot_pose = tools::pose_state(data_set.get_my_robot());
    const auto ball_pose = tools::pose_state(data_set.get_ball());

    const tools::Trans trans_BtoR(ball_pose, tools::calc_angle(ball_pose, robot_pose));
    const auto new_pose = trans_BtoR.inverted_transform(ROBOT_RADIUS * 0.0, 0.0, -M_PI);

    data_set.set_parsed_pose(new_pose);
    data_set.set_parsed_dribble_power(DRIBBLE_CATCH);

    const auto robot_id = data_set.get_my_robot().robot_id.id;
    const auto elapsed = chrono::duration<double>(chrono::system_clock::now() - tactic_time_[robot_id]).count();

    // if (tools::is_same(robot_pose, new_pose, 0.1, tools::to_radians(5.0)) &&
    if( elapsed > CATCHING_TIME) {
      std::cout << "Catched" << std::endl;
      return CARRY;
    }

    std::cout << "Catching" << std::endl;
    std::cout << "elapsed: " << elapsed << std::endl;
    
    return CATCH;
  };

  tactic_functions_[CARRY] = [this](TacticDataSet & data_set) -> TacticName {
    // Debug

    const auto robot_pose = tools::pose_state(data_set.get_my_robot());
    data_set.set_parsed_pose(robot_pose);
    data_set.set_parsed_dribble_power(DRIBBLE_RELEASE);

    const auto robot_id = data_set.get_my_robot().robot_id.id;
    const auto elapsed = chrono::duration<double>(chrono::system_clock::now() - tactic_time_[robot_id]).count();
    if (elapsed > 5.0) {
      return APPROACH;
    }

    std::cout << "Carrying" << std::endl;

    return CARRY;
  };

  tactic_functions_["RELEASE"] = [this](TacticDataSet & data_set) -> TacticName {
    std::cout << "Releasing the ball" << std::endl;
    return "FINISH";
  };

  tactic_functions_["FINISH"] = [this](TacticDataSet & data_set) -> TacticName {
    std::cout << "Dribbling is finished." << std::endl;
    return "FINISH";
  };
}

bool DribbleTactics::update(
  const State & dribble_target, const TrackedRobot & my_robot, const TrackedBall & ball,
  State & parsed_pose, double & parsed_dribble_power)
{
  const auto robot_id = my_robot.robot_id.id;
  // If the robot is not in the map, initialize the tactic state.
  if (tactic_name_.find(robot_id) == tactic_name_.end()) {
    tactic_name_[robot_id] = APPROACH;
  }

  // Execute the tactic fuction.
  TacticDataSet data_set(my_robot, ball, dribble_target, parsed_pose, 0.0, parsed_dribble_power);
  const auto next_tactic = tactic_functions_[tactic_name_[robot_id]](data_set);

  if (tactic_name_[robot_id] != next_tactic) {
    // Reset timestamp
    tactic_name_[robot_id] = next_tactic;
    tactic_time_[robot_id] = chrono::system_clock::now();
  }

  parsed_pose = data_set.get_parsed_pose();
  parsed_dribble_power = data_set.get_parsed_dribble_power();

  return true;
}

}  // namespace dribble_tactics
