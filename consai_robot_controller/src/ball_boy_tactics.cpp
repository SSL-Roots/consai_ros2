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

#include "consai_robot_controller/ball_boy_tactics.hpp"
#include "consai_robot_controller/geometry_tools.hpp"

namespace tactics
{

namespace tools = geometry_tools;

static constexpr double DRIBBLE_CATCH = 1.0;
static constexpr double DRIBBLE_RELEASE = 0.0;
static constexpr double ROBOT_RADIUS = 0.180 * 0.5;
static constexpr double DISTANCE_TO_CATCHER = 0.18;  // ロボット中心からボールキャッチャーまでの距離
static const State FIELD_CENTER;

BallBoyTactics::BallBoyTactics()
{
  tactic_functions_[TacticState::APPROACH] = [this](DataSet & data_set) {
      return tactic_approach(data_set);
    };
  tactic_functions_[TacticState::CATCH] = [this](DataSet & data_set) {
      return tactic_catch(data_set);
    };
  tactic_functions_[TacticState::CARRY] = [this](DataSet & data_set) {
      return tactic_carry(data_set);
    };
  tactic_functions_[TacticState::RELEASE] = [this](DataSet & data_set) {
      return tactic_release(data_set);
    };
}

bool BallBoyTactics::update(
  const State & dribble_target, const TrackedRobot & my_robot, const TrackedBall & ball,
  State & parsed_pose, double & parsed_dribble_power)
{
  const auto robot_id = my_robot.robot_id.id;
  if (tactic_state_.find(robot_id) == tactic_state_.end()) {
    tactic_state_[robot_id] = TacticState::APPROACH;
  }

  if (need_reset_state(my_robot, ball)) {
    tactic_state_[robot_id] = TacticState::APPROACH;
  }

  DataSet data_set(parsed_pose, dribble_target, my_robot, ball);
  tactic_state_[robot_id] = tactic_functions_[tactic_state_[robot_id]](data_set);

  parsed_pose = data_set.get_parsed_pose();
  parsed_dribble_power = data_set.get_parsed_dribble_power();

  return true;
}

bool BallBoyTactics::need_reset_state(const TrackedRobot & my_robot, const TrackedBall & ball) const
{
  // 状態リセットが必要化判定する
  const double RESET_DISTANCE = 0.5;

  // ボールがロボットから離れたらリセットする
  if (tools::distance(tools::pose_state(my_robot), tools::pose_state(ball)) > RESET_DISTANCE) {
    return true;
  }

  return false;
}

bool BallBoyTactics::is_same(const State & target, const State & current) const
{
  constexpr double THRESHOLD_DISTANCE = ROBOT_RADIUS * 0.5;
  const double THRESHOLD_ANGLE = tools::to_radians(10.0);

  if (tools::distance(target, current) < THRESHOLD_DISTANCE &&
    std::fabs(tools::normalize_theta(target.theta - current.theta)) < THRESHOLD_ANGLE)
  {
    return true;
  }

  return false;
}

TacticState BallBoyTactics::tactic_approach(DataSet & data_set) const
{
  // フィールド中心を背中にして、ボールに近づく
  const auto robot_pose = tools::pose_state(data_set.get_my_robot());
  const auto ball_pose = tools::pose_state(data_set.get_ball());

  const auto angle_ball_to_center = tools::calc_angle(ball_pose, FIELD_CENTER);
  const tools::Trans trans_BtoC(ball_pose, angle_ball_to_center);
  const auto new_parsed_pose = trans_BtoC.inverted_transform(DISTANCE_TO_CATCHER, 0.0, -M_PI);
  data_set.set_parsed_pose(new_parsed_pose);
  data_set.set_parsed_dribble_power(DRIBBLE_RELEASE);

  if (is_same(new_parsed_pose, robot_pose)) {
    return TacticState::CATCH;
  }

  return TacticState::APPROACH;
}

TacticState BallBoyTactics::tactic_catch(DataSet & data_set) const
{
  // キャッチャーを駆動させながら前進する
  constexpr double DISTANCE_TO_CATCH = DISTANCE_TO_CATCHER * 0.9;
  const auto robot_pose = tools::pose_state(data_set.get_my_robot());
  const auto ball_pose = tools::pose_state(data_set.get_ball());

  const auto angle_ball_to_center = tools::calc_angle(ball_pose, FIELD_CENTER);
  const tools::Trans trans_BtoC(ball_pose, angle_ball_to_center);
  const auto new_parsed_pose = trans_BtoC.inverted_transform(DISTANCE_TO_CATCH, 0.0, -M_PI);
  data_set.set_parsed_pose(new_parsed_pose);
  data_set.set_parsed_dribble_power(DRIBBLE_CATCH);

  if (is_same(new_parsed_pose, robot_pose)) {
    return TacticState::CARRY;
  }

  return TacticState::CATCH;
}

TacticState BallBoyTactics::tactic_carry(DataSet & data_set) const
{
  // 目標位置へボールを運ぶ
  const auto robot_pose = tools::pose_state(data_set.get_my_robot());
  const auto ball_pose = tools::pose_state(data_set.get_ball());
  const auto dribble_target = data_set.get_dribble_target();

  const auto angle_target_to_robot = tools::calc_angle(dribble_target, robot_pose);
  const tools::Trans trans_TtoR(dribble_target, angle_target_to_robot);
  const auto new_parsed_pose = trans_TtoR.inverted_transform(DISTANCE_TO_CATCHER, 0.0, -M_PI);

  data_set.set_parsed_pose(new_parsed_pose);
  data_set.set_parsed_dribble_power(DRIBBLE_CATCH);

  if (is_same(new_parsed_pose, robot_pose)) {
    return TacticState::RELEASE;
  }

  return TacticState::CARRY;
}

TacticState BallBoyTactics::tactic_release(DataSet & data_set) const
{
  // ボールを離す
  const double DISTANCE_TO_RELEASE = DISTANCE_TO_CATCHER + 0.0;
  const auto robot_pose = tools::pose_state(data_set.get_my_robot());
  const auto ball_pose = tools::pose_state(data_set.get_ball());
  const auto dribble_target = data_set.get_dribble_target();

  const auto angle_target_to_robot = tools::calc_angle(dribble_target, robot_pose);
  const tools::Trans trans_TtoR(dribble_target, angle_target_to_robot);
  const auto new_parsed_pose = trans_TtoR.inverted_transform(DISTANCE_TO_RELEASE, 0.0, -M_PI);

  data_set.set_parsed_pose(new_parsed_pose);
  data_set.set_parsed_dribble_power(DRIBBLE_RELEASE);

  if (is_same(new_parsed_pose, robot_pose)) {
    return TacticState::RELEASE;
  }

  return TacticState::RELEASE;
}


}  // namespace tactics
