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

#include "consai_robot_controller/kick_tactics.hpp"
#include "consai_robot_controller/geometry_tools.hpp"

namespace kick_tactics
{

namespace tools = geometry_tools;

static constexpr double DRIBBLE_CATCH = 1.0;
static constexpr double DRIBBLE_RELEASE = 0.0;
static constexpr double KICK_POWER = 1.0;
static constexpr double ROBOT_RADIUS = 0.180 * 0.5;

KickTactics::KickTactics()
{
  tactic_functions_[TacticState::STOP_THE_BALL] = [this](DataSet & data_set) {
      return tactic_stop_the_ball(data_set);
    };
  tactic_functions_[TacticState::BEHIND_THE_BALL] = [this](DataSet & data_set) {
      return tactic_behind_the_ball(data_set);
    };
}

bool KickTactics::update(
  const State & kick_target, const TrackedRobot & my_robot, const TrackedBall & ball,
  State & parsed_pose, double & parsed_kick_power, double & parsed_dribble_power)
{
  // 外部から実行される関数
  // ロボット（ロボットID）が持つ現在の戦術（Tactic）状態に合わせた関数を実行する
  // 戦術状態は、各関数内で更新される
  // 演算結果の移動目標位置・姿勢はparsed_poseに、キック/ドリブルパワーはparsed_kick/dribble_powerに格納される

  const auto robot_id = my_robot.robot_id.id;
  if (tactic_state_.find(robot_id) == tactic_state_.end()) {
    tactic_state_[robot_id] = TacticState::STOP_THE_BALL;
  }

  if (need_reset_state(my_robot, ball)) {
    tactic_state_[robot_id] = TacticState::STOP_THE_BALL;
  }

  DataSet data_set(parsed_pose, kick_target, my_robot, ball);
  tactic_state_[robot_id] = tactic_functions_[tactic_state_[robot_id]](data_set);

  parsed_pose = data_set.get_parsed_pose();
  parsed_kick_power = data_set.get_parsed_kick_power();
  parsed_dribble_power = data_set.get_parsed_dribble_power();

  return true;
}

bool KickTactics::need_reset_state(const TrackedRobot & my_robot, const TrackedBall & ball) const
{
  // 状態リセットが必要か判定する
  const double RESET_DISTANCE = 0.5;

  // ボールがロボットから離れたらリセットする
  if (tools::distance(tools::pose_state(my_robot), tools::pose_state(ball)) > RESET_DISTANCE) {
    return true;
  }

  // ボール速度が大きかったらリセットする
  if (ball_is_moving(ball)) {
    return true;
  }

  return false;
}

bool KickTactics::is_same(
  const State & target, const State & current, const double threshold_distance) const
{
  const double THRESHOLD_ANGLE = tools::to_radians(10.0);

  if (tools::distance(target, current) < threshold_distance &&
    std::fabs(tools::normalize_theta(target.theta - current.theta)) < THRESHOLD_ANGLE)
  {
    return true;
  }

  return false;
}

bool KickTactics::ball_is_moving(const TrackedBall & ball) const
{
  const double THRESHOLD_VELOCITY = 0.1;  // m/s

  if (tools::distance(State(), tools::vel_state(ball)) > THRESHOLD_VELOCITY) {
    return true;
  }

  return false;
}

TacticState KickTactics::tactic_stop_the_ball(DataSet & data_set) const
{
  const auto AVOID_BALL_Y = 0.3;

  const auto robot_pose = tools::pose_state(data_set.get_my_robot());
  const auto ball_pose = tools::pose_state(data_set.get_ball());
  const auto ball_vel = tools::vel_state(data_set.get_ball());
  const auto ball_vel_norm = tools::distance(State(), ball_vel);

  // ボール速度軌道上に移動する
  const auto angle_ball_vel = tools::calc_angle(State(), ball_vel);
  const tools::Trans trans_BtoV(ball_pose, angle_ball_vel);
  const auto robot_pose_BtoV = trans_BtoV.transform(robot_pose);

  auto target_x = std::max(2.0 * ball_vel_norm, 2.0);
  // ボール軌道の裏にロボットがいる場合は、ボール軌道上に回り込む
  auto target_y = std::copysign(AVOID_BALL_Y, robot_pose_BtoV.y);
  if (robot_pose_BtoV.x > ROBOT_RADIUS) {
    target_y = 0.0;
    target_x = 0.0;
  }

  const auto new_parsed_pose = trans_BtoV.inverted_transform(
    target_x, target_y, -M_PI);

  data_set.set_parsed_pose(new_parsed_pose);
  data_set.set_parsed_dribble_power(DRIBBLE_CATCH);

  // ボールが止まったら次の状態へ
  if (!ball_is_moving(data_set.get_ball())) {
    return TacticState::BEHIND_THE_BALL;
  }

  return TacticState::STOP_THE_BALL;
}

TacticState KickTactics::tactic_behind_the_ball(DataSet & data_set) const
{
  const auto robot_pose = tools::pose_state(data_set.get_my_robot());

  auto new_parsed_pose = robot_pose;
  new_parsed_pose.theta += tools::to_radians(5);

  data_set.set_parsed_pose(new_parsed_pose);
  data_set.set_parsed_dribble_power(DRIBBLE_RELEASE);
  return TacticState::BEHIND_THE_BALL;
}


}  // namespace kick_tactics
