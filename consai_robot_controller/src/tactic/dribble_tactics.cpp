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

namespace dribble_tactics
{

namespace tools = geometry_tools;
namespace chrono = std::chrono;

static constexpr double ROBOT_RADIUS = 0.09;  // ロボットの半径
static constexpr double DRIBBLE_CATCH = 1.0;
static constexpr double DRIBBLE_RELEASE = 0.0;
static constexpr auto APPROACH = "APPROACH";
// static constexpr auto CATCH = "CATCH";
static constexpr auto ROTATE = "ROTATE";
static constexpr auto CARRY = "CARRY";
static constexpr auto RELEASE = "RELEASE";


DribbleTactics::DribbleTactics()
{
  tactic_functions_[APPROACH] = [this](TacticDataSet & data_set) -> TacticName {
      // 現在位置からボールに対してまっすぐ進む
      constexpr auto DISTANCE_THRESHOLD = 0.1;  // meters
      const auto THETA_THRESHOLD = tools::to_radians(5.0);

      const auto robot_pose = tools::pose_state(data_set.get_my_robot());
      const auto ball_pose = tools::pose_state(data_set.get_ball());

      const tools::Trans trans_BtoR(ball_pose, tools::calc_angle(ball_pose, robot_pose));
      const auto new_pose = trans_BtoR.inverted_transform(ROBOT_RADIUS * 4.0, 0.0, -M_PI);

      data_set.set_parsed_pose(new_pose);
      data_set.set_parsed_dribble_power(DRIBBLE_RELEASE);

      if (tools::is_same(robot_pose, new_pose, DISTANCE_THRESHOLD, THETA_THRESHOLD)) {
        return ROTATE;
      }

      return APPROACH;
    };

  // tactic_functions_[CATCH] = [this](TacticDataSet & data_set) -> TacticName {
  //     // ドリブラーを駆動させながら前進する
  //     // ボールをキャッチするため、一定時間は行動を継続する
  //     constexpr auto CATCHING_TIME = 1.0;  // seconds

  //     const auto robot_pose = tools::pose_state(data_set.get_my_robot());
  //     const auto ball_pose = tools::pose_state(data_set.get_ball());

  //     const tools::Trans trans_BtoR(ball_pose, tools::calc_angle(ball_pose, robot_pose));
  //     const auto new_pose = trans_BtoR.inverted_transform(ROBOT_RADIUS * 0.0, 0.0, -M_PI);

  //     data_set.set_parsed_pose(new_pose);
  //     data_set.set_parsed_dribble_power(DRIBBLE_CATCH);

  //     const auto robot_id = data_set.get_my_robot().robot_id.id;
  //     const auto elapsed =
  //       chrono::duration<double>(chrono::system_clock::now() - tactic_time_[robot_id]).count();

  //     if (elapsed > CATCHING_TIME) {
  //       return ROTATE;
  //     }

  //     return CATCH;
  //   };

  tactic_functions_[ROTATE] = [this](TacticDataSet & data_set) -> TacticName {
      // ボールを中心にロボットが回転移動し、ボールと目標値の直線上に移動する
      // 目的位置をしっかり狙ったら、次の行動に移行する
      constexpr auto DISTANCE_THRESHOLD = 0.05;  // meters
      const auto THETA_THRESHOLD = tools::to_radians(5.0);
      const auto OMEGA_THRESHOLD = 0.1;  // rad/s

      const auto robot_pose = tools::pose_state(data_set.get_my_robot());
      const auto ball_pose = tools::pose_state(data_set.get_ball());
      const auto target_pose = data_set.get_target();

      // ボールから目標位置を結ぶ直線上で、ロボットがボールを見ながら、ボールの周りを旋回する
      const tools::Trans trans_BtoT(ball_pose, tools::calc_angle(ball_pose, target_pose));

      const auto robot_pose_BtoT = trans_BtoT.transform(robot_pose);
      const auto angle_robot_position = tools::calc_angle(State(), robot_pose_BtoT);

      const auto ADD_ANGLE = tools::to_radians(25.0);
      const auto AIM_ANGLE_THRETHOLD = tools::to_radians(10.0);

      double rotation_radius = ROBOT_RADIUS * 4.0;

      State new_pose;
      if (std::fabs(angle_robot_position) + ADD_ANGLE > M_PI - AIM_ANGLE_THRETHOLD) {
        // ボールの裏に回ったら、直進する
        new_pose = trans_BtoT.inverted_transform(-ROBOT_RADIUS, 0.0, 0.0);
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
        robot_vel.theta < OMEGA_THRESHOLD)
      {
        return CARRY;
      }

      return ROTATE;
    };

  tactic_functions_[CARRY] = [this](TacticDataSet & data_set) -> TacticName {
      // ボールが目的地に着くまで前進する
      constexpr auto DISTANCE_THRESHOLD = 0.01;  // meters
      const auto THETA_THRESHOLD = tools::to_radians(5.0);

      const auto robot_pose = tools::pose_state(data_set.get_my_robot());
      const auto target_pose = data_set.get_target();

      // ボールが消えることを考慮して、ターゲットとロボットの座標系で目標位置を生成する
      constexpr double MOVE_DISTANCE = 0.3;  //meters
      State new_pose;

      if (tools::distance(robot_pose, target_pose) > MOVE_DISTANCE) {
        const tools::Trans trans_RtoT(robot_pose, tools::calc_angle(robot_pose, target_pose));
        new_pose = trans_RtoT.inverted_transform(MOVE_DISTANCE, 0.0, 0.0);
      } else {
        // target付近で暴れないように、targetを基準にした目標位置を生成する
        const tools::Trans trans_TtoR(target_pose, tools::calc_angle(target_pose, robot_pose));
        new_pose = trans_TtoR.inverted_transform(ROBOT_RADIUS * 1.0, 0.0, -M_PI);
      }

      data_set.set_parsed_pose(new_pose);
      data_set.set_parsed_dribble_power(DRIBBLE_CATCH);

      if (tools::is_same(robot_pose, new_pose, DISTANCE_THRESHOLD, THETA_THRESHOLD)) {
        return RELEASE;
      }

      return CARRY;
    };

  tactic_functions_[RELEASE] = [this](TacticDataSet & data_set) -> TacticName {
      // ドリブラーをオフし、その場にとどまる
      const auto robot_pose = tools::pose_state(data_set.get_my_robot());
      const auto target_pose = data_set.get_target();
      const tools::Trans trans_TtoR(target_pose, tools::calc_angle(target_pose, robot_pose));
      const auto new_pose = trans_TtoR.inverted_transform(ROBOT_RADIUS * 1.0, 0.0, -M_PI);
      data_set.set_parsed_pose(new_pose);
      data_set.set_parsed_dribble_power(DRIBBLE_RELEASE);


      // ボールがtargetから大きく離れていたらリセット
      const auto ball_pose = tools::pose_state(data_set.get_ball());
      if (tools::distance(ball_pose, target_pose) > 0.05) {
        return APPROACH;
      }

      return RELEASE;
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

  // ロボットボールが大きく離れたらリセットする
  if (tools::distance(tools::pose_state(my_robot), tools::pose_state(ball)) > 0.5) {
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
