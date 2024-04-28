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

#include <iostream>

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
static constexpr double MIN_SHOOT_SPEED = 2.0;  // m/s
static constexpr double WAIT_DISTANCE = 0.7;  // meters
static constexpr double ROTATE_RADIUS = ROBOT_RADIUS * 2.0;  // meters
static constexpr double BALL_RADIUS = 0.0215;  // meters
static constexpr double MAX_BALL_VEL_NORM = 0.5;  // m/s
static constexpr auto WAIT = "WAIT";
static constexpr auto APPROACH = "APPROACH";
static constexpr auto ROTATE = "ROTATE";
static constexpr auto SHOOT = "SHOOT";
static constexpr auto APPROACH_TO_MOVING_BALL = "APPROACH_TO_MOVING_BALL";


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

      // セットプレイ時は回転半径を大きくする
      double rotation_radius = ROTATE_RADIUS;
      if (data_set.is_setplay()) {
        rotation_radius = ROBOT_RADIUS * 4.0;
      }

      const tools::Trans trans_BtoR(ball_pose, tools::calc_angle(ball_pose, robot_pose));
      const auto new_pose = trans_BtoR.inverted_transform(rotation_radius, 0.0, -M_PI);

      data_set.set_parsed_pose(new_pose);
      data_set.set_parsed_dribble_power(DRIBBLE_RELEASE);

      if (tools::is_same(robot_pose, new_pose, DISTANCE_THRESHOLD, THETA_THRESHOLD)) {
        return ROTATE;
      }

      return APPROACH;
    };

tactic_functions_[APPROACH_TO_MOVING_BALL] = [this](TacticDataSet & data_set) -> TacticName {
      // 現在位置からボールに対してまっすぐ進む

      // 近づけば良いので、しきい値を大きくする
      constexpr auto DISTANCE_THRESHOLD = 0.3;  // meters
      const auto THETA_THRESHOLD = tools::to_radians(5.0);
      const auto GO_AROUND_BALL_X = 0.2; // meters
      const auto GO_AROUND_BALL_Y = 0.2; // meters

      const auto robot_pose = tools::pose_state(data_set.get_my_robot());
      const auto ball_pose = tools::pose_state(data_set.get_ball());

      const auto ball_vel = tools::velocity_ball_state(data_set.get_ball());

      const tools::Trans trans_BtoBV(ball_pose, std::atan2(ball_vel.y, ball_vel.x));
      const auto robot_pose_BVtoB = trans_BtoBV.transform(robot_pose);
      const auto ball_velocity_norm = std::hypot(ball_vel.x, ball_vel.y);
      if (ball_velocity_norm < MAX_BALL_VEL_NORM){
        return ROTATE;
      }

      double go_around_ball_y = GO_AROUND_BALL_Y;
      if (robot_pose_BVtoB.y < 0) {
        go_around_ball_y = -GO_AROUND_BALL_Y;
      }

      const auto new_pose = trans_BtoBV.inverted_transform(GO_AROUND_BALL_X, go_around_ball_y, 0);

      // セットプレイ時は回転半径を大きくする
      double rotation_radius = ROTATE_RADIUS;
      if (data_set.is_setplay()) {
        rotation_radius = ROBOT_RADIUS * 4.0;
      }

      data_set.set_parsed_pose(new_pose);
      data_set.set_parsed_dribble_power(DRIBBLE_RELEASE);

      if (tools::is_same(robot_pose, new_pose, DISTANCE_THRESHOLD, THETA_THRESHOLD)) {
        return ROTATE;
      }

      return APPROACH_TO_MOVING_BALL;
    };


  tactic_functions_[ROTATE] = [this](TacticDataSet & data_set) -> TacticName {
      // ボールを中心にロボットが回転移動し、ボールと目標値の直線上に移動する
      // 目的位置をしっかり狙ったら、次の行動に移行する
      constexpr auto DISTANCE_THRESHOLD = 0.05;  // meters
      const auto THETA_THRESHOLD = tools::to_radians(5.0);
      const auto OMEGA_THRESHOLD = 0.5;  // rad/s
      const auto OMEGA_THRESHOLD_FOR_SETPLAY = 0.01;  // rad/s

      const auto robot_pose = tools::pose_state(data_set.get_my_robot());
      const auto ball_pose = tools::pose_state(data_set.get_ball());
      const auto target_pose = data_set.get_target();

      // ボールから目標位置を結ぶ直線上で、ロボットがボールを見ながら、ボールの周りを旋回する
      const tools::Trans trans_BtoT(ball_pose, tools::calc_angle(ball_pose, target_pose));

      const auto robot_pose_BtoT = trans_BtoT.transform(robot_pose);
      const auto angle_robot_position = tools::calc_angle(State(), robot_pose_BtoT);

      const auto ADD_ANGLE = tools::to_radians(50.0);
      // FIXME: 下2つとaim_angle_thresholdは不要だったら削除
      // const auto AIM_ANGLE_THRETHOLD = tools::to_radians(20.0);
      // const auto AIM_ANGLE_THRETHOLD_FOR_SETPLAY = tools::to_radians(10.0);

      // セットプレイ時は回転半径を大きくする
      double rotation_radius = ROTATE_RADIUS;
      double forward_distance = ROBOT_RADIUS;
      // double aim_angle_threshold = AIM_ANGLE_THRETHOLD;
      double omega_threshold = OMEGA_THRESHOLD;
      if (data_set.is_setplay()) {
        rotation_radius = ROBOT_RADIUS * 4.0;
        forward_distance = ROBOT_RADIUS * 2.0;
        // aim_angle_threshold = AIM_ANGLE_THRETHOLD_FOR_SETPLAY;
        omega_threshold = OMEGA_THRESHOLD_FOR_SETPLAY;
      }

      State new_pose;
      if (std::fabs(angle_robot_position) + ADD_ANGLE > M_PI) {
        // ボールの裏に回ったら、直進する
        new_pose = trans_BtoT.inverted_transform(-forward_distance, 0.0, 0.0);
      } else {
        // ボールの周りを旋回する
        const auto theta = angle_robot_position + std::copysign(ADD_ANGLE, angle_robot_position);
        double pos_x = rotation_radius * std::cos(theta);
        double pos_y = rotation_radius * std::sin(theta);
        new_pose = trans_BtoT.inverted_transform(pos_x, pos_y, theta + M_PI);
      }

      data_set.set_parsed_pose(new_pose);
      data_set.set_parsed_dribble_power(DRIBBLE_CATCH);


      // ロボットが目標姿勢に近づき、回転速度が小さくなったら次の行動に移行
      const auto robot_vel = tools::velocity_state(data_set.get_my_robot());

      if (tools::is_same(robot_pose, new_pose, DISTANCE_THRESHOLD, THETA_THRESHOLD) &&
        std::fabs(robot_vel.theta) < omega_threshold)
      {
        // ロボットの移動がぶれないように、目標位置を固定する
        shoot_target_ = target_pose;
        return SHOOT;
      }

      return ROTATE;
    };

  tactic_functions_[SHOOT] = [this](TacticDataSet & data_set) -> TacticName {
      // ロボットからみてボールが正面にあるかのしきい値
      const auto THETA_THRESHOLD = tools::to_radians(30.0);
      // 目的位置に向かってシュートする
      // ボールが離れたら終了する
      const auto robot_pose = tools::pose_state(data_set.get_my_robot());
      const auto ball_pose = tools::pose_state(data_set.get_ball());

      // 目標位置をボールの奥に設定し、前進する
      const tools::Trans trans_BtoT(ball_pose, tools::calc_angle(ball_pose, shoot_target_));
      const auto new_pose = trans_BtoT.inverted_transform(BALL_RADIUS, 0.0, 0.0);

      // ロボットから見てボールが正面にない場合は
      // SHOOTを抜けて、APPROACHに戻る
      const tools::Trans trans_RtoB(ball_pose, tools::calc_angle(robot_pose, ball_pose));
      const auto robot_pose_RtoB = trans_RtoB.transform(robot_pose);
      if (std::fabs(robot_pose_RtoB.theta) > THETA_THRESHOLD) {
        return APPROACH;
      }

      double shoot_speed = MAX_SHOOT_SPEED;
      if (data_set.is_pass()) {
        const auto distance = tools::distance(ball_pose, shoot_target_);
        const double ALPHA = 1.3;
        shoot_speed = std::clamp(distance * ALPHA, MIN_SHOOT_SPEED, MAX_SHOOT_SPEED);
      }

      data_set.set_parsed_pose(new_pose);
      data_set.set_parsed_dribble_power(DRIBBLE_CATCH);
      data_set.set_parsed_kick_power(shoot_speed);

      if (tools::distance(robot_pose, ball_pose) > ROTATE_RADIUS * 2.0) {
        return WAIT;
      }

      return SHOOT;
    };
}

bool ShootTactics::update(
  const State & shoot_target, const TrackedRobot & my_robot, const TrackedBall & ball,
  const bool & is_pass, const bool & is_setplay,
  State & parsed_pose, double & parsed_kick_power, double & parsed_dribble_power)
{
  const auto robot_id = my_robot.robot_id.id;
  // If the robot is not in the map, initialize the tactic state.
  if (tactic_name_.find(robot_id) == tactic_name_.end()) {
    tactic_name_[robot_id] = WAIT;
    tactic_time_[robot_id] = chrono::system_clock::now();
  }

  const auto robot_pose = tools::pose_state(my_robot);
  const auto ball_pose = tools::pose_state(ball);
  const auto ball_vel = tools::velocity_ball_state(ball);
  const auto ball_velocity_norm = std::hypot(ball_vel.x, ball_vel.y);

  const tools::Trans trans_BtoBV(ball_pose, std::atan2(ball_vel.y, ball_vel.x));
  const auto robot_pose_BtoBV = trans_BtoBV.transform(robot_pose);

  // ボールがロボットから離れている方向
  // ボール速度がある程度大きい
  // ボール速度は自陣側（シュートする反対側）
  if (robot_pose_BtoBV.x < 0 && ball_velocity_norm > MAX_BALL_VEL_NORM && ball_vel.x < 0) {
    // ボール進行方向に回り込む
    tactic_name_[robot_id] = APPROACH_TO_MOVING_BALL;
    tactic_time_[robot_id] = chrono::system_clock::now();
  }  

  // ロボットボールが大きく離れたらリセットする
  if (tools::distance(tools::pose_state(my_robot), tools::pose_state(ball)) > WAIT_DISTANCE && ball_velocity_norm <= MAX_BALL_VEL_NORM) {
    tactic_name_[robot_id] = WAIT;
    tactic_time_[robot_id] = chrono::system_clock::now();
  }

  // Execute the tactic fuction.
  TacticDataSet data_set(my_robot, ball, shoot_target, parsed_pose, parsed_kick_power,
    parsed_dribble_power);
  data_set.set_pass(is_pass);
  data_set.set_setplay(is_setplay);
  const auto next_tactic = tactic_functions_[tactic_name_[robot_id]](data_set);

  if (tactic_name_[robot_id] != next_tactic) {
    // Reset timestamp
    tactic_name_[robot_id] = next_tactic;
    tactic_time_[robot_id] = chrono::system_clock::now();
  }

  // FIXME: デバッグ用なので消してね
  std::cout << tactic_name_[robot_id] << std::endl;

  parsed_pose = data_set.get_parsed_pose();
  parsed_dribble_power = data_set.get_parsed_dribble_power();
  parsed_kick_power = data_set.get_parsed_kick_power();

  return true;
}

}  // namespace shoot_tactics
