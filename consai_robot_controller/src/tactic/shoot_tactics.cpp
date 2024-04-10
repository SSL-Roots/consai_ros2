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

#include "consai_robot_controller/tactic/shoot_tactics.hpp"
#include "consai_tools/geometry_tools.hpp"

namespace shoot_tactics
{

namespace tools = geometry_tools;
namespace chrono = std::chrono;

static constexpr double ROBOT_RADIUS = 0.09;  // ロボットの半径
static constexpr double DRIBBLE_CATCH = 1.0;
static constexpr double DRIBBLE_RELEASE = 0.0;
static constexpr double MAX_SHOOT_SPEED = 5.5;  // m/s
static constexpr double WAIT_DISTANCE = 0.7;  // meters
static constexpr auto WAIT = "WAIT";
static constexpr auto APPROACH = "APPROACH";
static constexpr auto CATCH = "CATCH";
static constexpr auto ROTATE = "ROTATE";
static constexpr auto SHOOT = "SHOOT";


ShootTactics::ShootTactics()
{
  tactic_functions_[WAIT] = [this](TacticDataSet & data_set) -> TacticName {
    // ボールが近づいてくるまで待機
    // キックのための必須処理ではないので、削除してもOK
    const auto robot_pose = tools::pose_state(data_set.get_my_robot());
    const auto ball_pose = tools::pose_state(data_set.get_ball());

    if (tools::distance(robot_pose, ball_pose) <= WAIT_DISTANCE) {
      return APPROACH;
    }

    return WAIT;
  };

  tactic_functions_[APPROACH] = [this](TacticDataSet & data_set) -> TacticName {
    // 現在位置からボールに対してまっすぐ進む

    // 近づけば良いので、しきい値を大きくする
    constexpr auto DISTANCE_THRESHOLD = 0.3;  // meters
    const auto THETA_THRESHOLD = tools::to_radians(5.0);

    const auto robot_pose = tools::pose_state(data_set.get_my_robot());
    const auto ball_pose = tools::pose_state(data_set.get_ball());

    const tools::Trans trans_BtoR(ball_pose, tools::calc_angle(ball_pose, robot_pose));
    const auto new_pose = trans_BtoR.inverted_transform(ROBOT_RADIUS * 2.0, 0.0, -M_PI);

    data_set.set_parsed_pose(new_pose);
    data_set.set_parsed_dribble_power(DRIBBLE_RELEASE);

    if (tools::is_same(robot_pose, new_pose, DISTANCE_THRESHOLD, THETA_THRESHOLD)) {
      return CATCH;
    }

    return APPROACH;
  };

  tactic_functions_[CATCH] = [this](TacticDataSet & data_set) -> TacticName {
    // ドリブラーを駆動させながら前進する
    // ボールをキャッチするため、一定時間は行動を継続する
    constexpr auto CATCHING_TIME = 0.3;  // seconds

    const auto robot_pose = tools::pose_state(data_set.get_my_robot());
    const auto ball_pose = tools::pose_state(data_set.get_ball());

    const tools::Trans trans_BtoR(ball_pose, tools::calc_angle(ball_pose, robot_pose));
    const auto new_pose = trans_BtoR.inverted_transform(ROBOT_RADIUS * 0.0, 0.0, -M_PI);

    data_set.set_parsed_pose(new_pose);
    data_set.set_parsed_dribble_power(DRIBBLE_CATCH);

    const auto robot_id = data_set.get_my_robot().robot_id.id;
    const auto elapsed =
      chrono::duration<double>(chrono::system_clock::now() - tactic_time_[robot_id]).count();

    if (elapsed > CATCHING_TIME) {
      return ROTATE;
    }

    return CATCH;
  };

  tactic_functions_[ROTATE] = [this](TacticDataSet & data_set) -> TacticName {
    // ボールを中心にロボットが回転移動し、ボールと目標値の直線上に移動する
    // 目的位置をしっかり狙ったら、次の行動に移行する
    constexpr auto DISTANCE_THRESHOLD = 0.05;  // meters
    const auto THETA_THRESHOLD = tools::to_radians(3.0);

    const auto robot_pose = tools::pose_state(data_set.get_my_robot());
    const auto ball_pose = tools::pose_state(data_set.get_ball());
    const auto target_pose = data_set.get_target();

    const tools::Trans trans_BtoT(ball_pose, tools::calc_angle(ball_pose, target_pose));
    const auto new_pose = trans_BtoT.inverted_transform(-ROBOT_RADIUS, 0.0, 0.0);

    data_set.set_parsed_pose(new_pose);
    data_set.set_parsed_dribble_power(DRIBBLE_CATCH);

    if (tools::is_same(robot_pose, new_pose, DISTANCE_THRESHOLD, THETA_THRESHOLD)) {
      return SHOOT;
    }

    return ROTATE;
  };

  tactic_functions_[SHOOT] = [this](TacticDataSet & data_set) -> TacticName {
    // 目的位置に向かってシュートする
    // ボールが離れたら終了する
    const auto robot_pose = tools::pose_state(data_set.get_my_robot());
    const auto ball_pose = tools::pose_state(data_set.get_ball());

    data_set.set_parsed_dribble_power(DRIBBLE_RELEASE);
    data_set.set_parsed_kick_power(MAX_SHOOT_SPEED);

    if (tools::distance(robot_pose, ball_pose) > ROBOT_RADIUS * 2.0) {
      return WAIT;
    }

    return SHOOT;
  };
}

bool ShootTactics::update(
  const State & shoot_target, const TrackedRobot & my_robot, const TrackedBall & ball,
  const bool & is_pass,
  State & parsed_pose, double & parsed_kick_power, double & parsed_dribble_power)
{
  const auto robot_id = my_robot.robot_id.id;
  // If the robot is not in the map, initialize the tactic state.
  if (tactic_name_.find(robot_id) == tactic_name_.end()) {
    tactic_name_[robot_id] = WAIT;
    tactic_time_[robot_id] = chrono::system_clock::now();
  }

  // ロボットボールが大きく離れたらリセットする
  if (tools::distance(tools::pose_state(my_robot), tools::pose_state(ball)) > WAIT_DISTANCE) {
    tactic_name_[robot_id] = WAIT;
    tactic_time_[robot_id] = chrono::system_clock::now();
  }

  // Execute the tactic fuction.
  TacticDataSet data_set(my_robot, ball, shoot_target, parsed_pose, parsed_kick_power, parsed_dribble_power);
  const auto next_tactic = tactic_functions_[tactic_name_[robot_id]](data_set);

  if (tactic_name_[robot_id] != next_tactic) {
    // Reset timestamp
    tactic_name_[robot_id] = next_tactic;
    tactic_time_[robot_id] = chrono::system_clock::now();
  }

  parsed_pose = data_set.get_parsed_pose();
  parsed_dribble_power = data_set.get_parsed_dribble_power();
  parsed_kick_power = data_set.get_parsed_kick_power();

  return true;
}

}  // namespace shoot_tactics
