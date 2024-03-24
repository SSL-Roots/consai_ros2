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


#include "consai_robot_controller/tactic_obstacle_avoidance.hpp"
#include "consai_robot_controller/geometry_tools.hpp"

namespace tactic
{

namespace tools = geometry_tools;

ObstacleAvoidance::ObstacleAvoidance(
  const std::shared_ptr<DetectionExtractor> & detection_extractor)
{
  detection_ = detection_extractor;
}

bool ObstacleAvoidance::avoid_obstacles(
  const TrackedRobot & my_robot, const State & goal_pose, const TrackedBall & ball,
  const bool & avoid_ball, State & avoidance_pose) const
{
  // 障害物を回避するposeを生成する
  // 全ロボット情報を検索し、
  // 自己位置(my_robot)と目標位置(goal_pose)間に存在する、
  // 自己位置に最も近い障害物付近に回避点を生成し、その回避点を新しい目標位置とする

  // 自身から直進方向に何m離れたロボットを障害物と判定するか
  const double OBSTACLE_DETECTION_X = 0.5;
  // 自身から直進方向に対して左右何m離れたロボットを障害物と判定するか
  const double OBSTACLE_DETECTION_Y_ROBOT = 0.3;
  const double OBSTACLE_DETECTION_Y_BALL = 0.2;
  // 回避の相対位置
  const double AVOIDANCE_POS_X_SHORT = 0.1;
  const double AVOIDANCE_POS_X_LONG = 0.2;
  const double AVOIDANCE_POS_Y = 0.4;

  // 相対的な回避位置
  double avoidance_pos_x = 0.0;
  double avoidance_pos_y = 0.0;

  // 障害物の検索ループの
  const int NUM_ITERATIONS = 6;

  // 自己位置の格納
  auto my_robot_pose = tools::pose_state(my_robot);
  // 回避位置の初期値を目標位置にする
  avoidance_pose = goal_pose;

  std::shared_ptr<State> obstacle_pose_MtoA;

  // 回避位置生成と回避位置-自己位置間の障害物を検索するためのループ
  for (auto iter = 0; iter < NUM_ITERATIONS; iter++) {
    // 各変数を更新
    double distance = 0.0;  // 距離を格納する変数
    double distance_to_obstacle = 10000;  // 自己位置と障害物間の距離(適当な大きい値を格納)
    bool need_avoid = false;  // 障害物の存在の判定フラグ

    // 座標をロボット-目標位置間の座標系に変換
    tools::Trans trans_MtoA(my_robot_pose, tools::calc_angle(my_robot_pose, avoidance_pose));
    auto avoidance_pose_MtoA = trans_MtoA.transform(avoidance_pose);
    auto goal_pose_MtoA = trans_MtoA.transform(goal_pose);

    // ロボットに対して回避位置を生成
    for (const auto & robot : detection_->extract_robots()) {
      // 自身の情報は除外する
      if (robot.robot_id.id == my_robot.robot_id.id &&
        robot.robot_id.team_color == my_robot.robot_id.team_color)
      {
        continue;
      }

      // ロボットが目標位置との間に存在するか判定
      auto robot_pose = tools::pose_state(robot);
      auto robot_pose_MtoA = trans_MtoA.transform(robot_pose);

      distance = std::hypot(robot_pose_MtoA.x, robot_pose_MtoA.y);
      if (0 < robot_pose_MtoA.x &&
        robot_pose_MtoA.x < avoidance_pose_MtoA.x &&
        std::fabs(robot_pose_MtoA.y) < OBSTACLE_DETECTION_Y_ROBOT)
      {
        if (distance < distance_to_obstacle) {
          obstacle_pose_MtoA = std::make_shared<State>(robot_pose_MtoA);
          distance_to_obstacle = distance;
          need_avoid = true;
        }
      }
    }

    // ボールが目標位置との間に存在するか判定
    if (avoid_ball) {
      auto ball_pose = tools::pose_state(ball);
      auto ball_pose_MtoA = trans_MtoA.transform(ball_pose);

      distance = std::hypot(ball_pose_MtoA.x, ball_pose_MtoA.y);
      // 進路上にボールエリアがある場合
      if (0 < ball_pose_MtoA.x &&
        ball_pose_MtoA.x < avoidance_pose_MtoA.x &&
        std::fabs(ball_pose_MtoA.y) < OBSTACLE_DETECTION_Y_BALL)
      {
        if (distance < distance_to_obstacle) {
          obstacle_pose_MtoA = std::make_shared<State>(ball_pose_MtoA);
          distance_to_obstacle = distance;
          need_avoid = true;
        }
      }
    }

    // 障害物が無い場合
    if (need_avoid == false) {
      // ループを抜ける
      break;
    } else {
      // 障害物がある場合
      // 相対的なY方向の回避位置を設定
      avoidance_pos_y = -std::copysign(AVOIDANCE_POS_Y, obstacle_pose_MtoA->y);
      // 障害物と距離が近い場合
      if (obstacle_pose_MtoA->x < OBSTACLE_DETECTION_X) {
        avoidance_pos_x = AVOIDANCE_POS_X_SHORT;
      } else {
        avoidance_pos_x = AVOIDANCE_POS_X_LONG;
      }

      // 回避位置を生成
      avoidance_pose = trans_MtoA.inverted_transform(
        obstacle_pose_MtoA->x + avoidance_pos_x,
        obstacle_pose_MtoA->y + avoidance_pos_y,
        goal_pose_MtoA.theta
      );
    }
  }
  return true;
}

}  // namespace tactic
