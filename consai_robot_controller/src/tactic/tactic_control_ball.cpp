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


#include "consai_robot_controller/tactic/tactic_control_ball.hpp"
#include "consai_tools/geometry_tools.hpp"

namespace tactic
{

namespace tools = geometry_tools;

void ControlBall::set_kick_power_params(
  double max_shoot_speed,
  double max_pass_speed,
  double min_pass_speed)
{
  max_shoot_speed_ = max_shoot_speed;
  max_pass_speed_ = max_pass_speed;
  min_pass_speed_ = min_pass_speed;
}

bool ControlBall::kick_ball(
  const State & target, const TrackedRobot & my_robot, const TrackedBall & ball,
  const bool is_pass, State & parsed_pose,
  double & parsed_kick_power, double & parsed_dribble_power) const
{
  if (!can_control_ball(ball, parsed_pose)) {
    return false;
  }

  const double DRIBBLE_DISTANCE = 0.0;  // TODO(ShotaAk): 位置を使わずに前進速度を決めたい
  return control_ball(
    target, my_robot, ball, DRIBBLE_DISTANCE, is_pass,
    parsed_pose, parsed_kick_power, parsed_dribble_power);
}

bool ControlBall::dribble_ball(
  const State & target, const TrackedRobot & my_robot, const TrackedBall & ball,
  State & parsed_pose, double & parsed_dribble_power) const
{
  if (!can_control_ball(ball, parsed_pose)) {
    return false;
  }

  const double DRIBBLE_DISTANCE = 0.15;  // TODO(ShotaAk): 位置を使わずに前進速度を決めたい
  const bool is_pass = false;
  double parsed_kick_power = 0.0;
  return control_ball(
    target, my_robot, ball, DRIBBLE_DISTANCE, is_pass,
    parsed_pose, parsed_kick_power, parsed_dribble_power);
}

bool ControlBall::kick_ball_at_setplay(
  const State & target, const TrackedRobot & my_robot, const TrackedBall & ball,
  const bool is_pass, State & parsed_pose,
  double & parsed_kick_power, double & parsed_dribble_power) const
{
  // セットプレイ用の落ち着いたキックを実施
  if (!can_control_ball(ball, parsed_pose)) {
    return false;
  }

  // 変数の初期化
  parsed_kick_power = 0.0;
  parsed_dribble_power = 0.0;

  auto ball_pose = tools::pose_state(ball);
  auto robot_pose = tools::pose_state(my_robot);

  // ボールからターゲットを見た座標系を生成
  auto angle_ball_to_target = tools::calc_angle(ball_pose, target);
  tools::Trans trans_BtoT(ball_pose, angle_ball_to_target);
  auto robot_pose_BtoT = trans_BtoT.transform(robot_pose);

  // ロボットがボールの裏側に回ったらcan_kick
  bool can_kick = robot_pose_BtoT.x < 0.01 && std::fabs(robot_pose_BtoT.y) < 0.05;

  if (can_kick) {
    parsed_pose = trans_BtoT.inverted_transform(0.02, 0.0, 0.0);
    parsed_kick_power = kick_speed(is_pass, my_robot, target);
  } else {
    parsed_pose = trans_BtoT.inverted_transform(-0.3, 0.0, 0.0);
  }
  return true;
}

bool ControlBall::receive_ball(
  const TrackedRobot & my_robot, const TrackedBall & ball,
  State & parsed_pose, double & parsed_dribble_power) const
{
  // 転がっているボールを受け取る

  // ボール情報に速度情報がなければ終了
  if (ball.vel.size() == 0) {
    return false;
  }

  State velocity;
  velocity.x = ball.vel[0].x;
  velocity.y = ball.vel[0].y;
  // ボール速度が一定値以下であれば終了
  if (std::hypot(velocity.x, velocity.y) <= 0.5) {
    return false;
  }

  auto ball_pose = tools::pose_state(ball);
  auto robot_pose = tools::pose_state(my_robot);
  auto angle_velocity = std::atan2(velocity.y, velocity.x);
  tools::Trans trans_BtoV(ball_pose, angle_velocity);

  auto robot_pose_BtoV = trans_BtoV.transform(robot_pose);

  // ボールの軌道から離れていたら終了
  if (std::fabs(robot_pose_BtoV.y) > 1.0 || robot_pose_BtoV.x < 0.0) {
    return false;
  }

  // ボールの軌道上に移動する
  robot_pose_BtoV.y = 0.0;
  robot_pose_BtoV.theta = M_PI;
  parsed_pose = trans_BtoV.inverted_transform(robot_pose_BtoV);
  parsed_dribble_power = max_dribble_power_;

  return true;
}

bool ControlBall::reflect_kick(
  const State & target, const TrackedRobot & my_robot, const TrackedBall & ball,
  const bool & kick_pass, State & parsed_pose, double & parsed_kick_power,
  double & parsed_dribble_power) const
{
  // 転がっているボールの軌道上に移動し、targetに向かって蹴る
  // targetを狙えない場合は、蹴らずにボールを受け取る

  const double MIN_VELOCITY_THRESHOLD = 0.5;  // m/s ボールの最低動作速度
  const double MAX_DISTANCE_TO_RECEIVE = 1.0;  // meters ボールを受け取る最長距離
  const double DISTANCE_TO_DRIBBLER = 0.055;  // meters ロボットの中心からドリブラーまでの距離
  const double CAN_REFLECT_ANGLE = 60.0;  // degress リフレクトできる最大角度

  // パラメータを初期化
  parsed_dribble_power = 0.0;
  parsed_kick_power = 0.0;

  // ボール情報に速度情報がなければ終了
  if (ball.vel.size() == 0) {
    return false;
  }

  State velocity;
  velocity.x = ball.vel[0].x;
  velocity.y = ball.vel[0].y;
  auto velocity_norm = std::hypot(velocity.x, velocity.y);
  // ボール速度が一定値以下であれば終了
  if (velocity_norm <= MIN_VELOCITY_THRESHOLD) {
    return false;
  }

  // ロボット座標と、ロボットのドリブラー座標を作成
  auto robot_pose = tools::pose_state(my_robot);
  auto dribbler_pose = robot_pose;
  dribbler_pose.x += DISTANCE_TO_DRIBBLER * std::cos(robot_pose.theta);
  dribbler_pose.y += DISTANCE_TO_DRIBBLER * std::sin(robot_pose.theta);

  // ボールを中心にボール速度方向への座標系を作成
  auto ball_pose = tools::pose_state(ball);
  auto angle_velocity = std::atan2(velocity.y, velocity.x);
  tools::Trans trans_BtoV(ball_pose, angle_velocity);

  auto dribbler_pose_BtoV = trans_BtoV.transform(dribbler_pose);

  // ロボットがボールの軌道から離れていたら終了
  if (std::fabs(dribbler_pose_BtoV.y) > MAX_DISTANCE_TO_RECEIVE || dribbler_pose_BtoV.x < 0.0) {
    return false;
  }

  // ドリブラーをボール軌道上へ移動する
  dribbler_pose_BtoV.y = 0.0;

  // targetへの角度を計算し、リフレクトシュートできるか判定する
  auto target_BtoV = trans_BtoV.transform(target);
  auto angle_dribbler_to_target_BtoV = tools::calc_angle(dribbler_pose_BtoV, target_BtoV);
  if (std::fabs(angle_dribbler_to_target_BtoV) < tools::to_radians(180 - CAN_REFLECT_ANGLE)) {
    // リフレクトシュートできないため終了
    return false;
  }

  auto receiving_dribbler_pose = trans_BtoV.inverted_transform(dribbler_pose_BtoV);
  auto angle_dribbler_to_target = tools::calc_angle(receiving_dribbler_pose, target);
  tools::Trans trans_DtoT(receiving_dribbler_pose, angle_dribbler_to_target);
  auto ball_pose_DtoT = trans_DtoT.transform(ball_pose);
  auto angle_dribbler_to_ball_DtoT = std::atan2(ball_pose_DtoT.y, ball_pose_DtoT.x);

  // リフレクトシュート目標位置を生成
  // TODO(Roots) :ボール速度、キック速度のベクトルを結合して、目標角度を求める
  auto target_angle_DtoT = angle_dribbler_to_ball_DtoT * 0.7 * velocity_norm / 6.5;
  parsed_pose = trans_DtoT.inverted_transform(-DISTANCE_TO_DRIBBLER, 0.0, target_angle_DtoT);
  // キックパワーをセット
  if (kick_pass) {
    parsed_kick_power = max_pass_speed_;
  } else {
    parsed_kick_power = max_shoot_speed_;
  }

  return true;
}

bool ControlBall::can_control_ball(const TrackedBall & ball, const State & parsed_pose) const
{
  const auto ball_pose = tools::pose_state(ball);
  if (tools::distance(ball_pose, parsed_pose) < can_control_distance_) {
    return true;
  }
  return false;
}

bool ControlBall::control_ball(
  const State & target, const TrackedRobot & my_robot, const TrackedBall & ball,
  const double & dribble_distance, const bool is_pass,
  State & parsed_pose,
  double & parsed_kick_power, double & parsed_dribble_power) const
{
  // ボールを操作する関数
  // キック、パス、ドリブルの操作が可能

  // 変数の初期化
  parsed_dribble_power = 0.0;
  parsed_kick_power = 0.0;

  auto ball_pose = tools::pose_state(ball);
  auto robot_pose = tools::pose_state(my_robot);

  // ボールの半径
  const double BALL_RADIUS = 0.043 * 0.5;
  // ロボットの半径
  const double ROBOT_RADIUS = 0.180 * 0.5;
  const double MAX_X = BALL_RADIUS + 0.2;
  const double MAX_Y = BALL_RADIUS + 0.2;

  // // ボールからターゲットを見た座標系を生成
  auto angle_ball_to_target = tools::calc_angle(ball_pose, target);
  tools::Trans trans_BtoT(ball_pose, angle_ball_to_target);
  auto robot_pose_BtoT = trans_BtoT.transform(robot_pose);

  // ロボットから見たボールまでの角度を算出
  auto angle_robot_to_ball_BtoT =
    tools::calc_angle(robot_pose_BtoT, trans_BtoT.transform(ball_pose));

  // ボールより前方にロボットが存在する場合
  if (0.0 < robot_pose_BtoT.x) {
    // ボールの斜め後ろに目標座標を設定
    parsed_pose = trans_BtoT.inverted_transform(
      -MAX_X, std::copysign(MAX_Y, robot_pose_BtoT.y), angle_robot_to_ball_BtoT);
  } else if (BALL_RADIUS < std::fabs(robot_pose_BtoT.y)) {
    // ボールの後ろにロボットが存在しない場合
    // ボールの後ろに目標座標を設定
    //   角度はボールの方向を向く
    parsed_pose = trans_BtoT.inverted_transform(-MAX_X, 0.0, angle_robot_to_ball_BtoT);
  } else {
    // ボールの後ろにロボットが存在する場合
    // ドリブルON
    parsed_dribble_power = max_dribble_power_;
    // ボールのすぐ後ろに目標座標を設定
    parsed_pose = trans_BtoT.inverted_transform(-BALL_RADIUS, 0.0, 0.0);
    if (std::fabs(angle_robot_to_ball_BtoT) < tools::to_radians(3.0)) {
      // ロボットが目標座標の方向を向いている場合
      parsed_kick_power = kick_speed(is_pass, my_robot, target);
    }
  }
  return true;
}


double ControlBall::kick_speed(
  const bool is_pass, const TrackedRobot & my_robot, const State & target) const
{
  const double MIN_PASS_DISTANCE = 1.0;  // meters
  if (!is_pass) {
    return max_shoot_speed_;
  }

  const auto robot_pose = tools::pose_state(my_robot);
  const double distance = tools::distance(robot_pose, target);
  if (distance < MIN_PASS_DISTANCE) {
    return min_pass_speed_;
  }

  return std::min(distance, max_pass_speed_);
}


}  // namespace tactic
