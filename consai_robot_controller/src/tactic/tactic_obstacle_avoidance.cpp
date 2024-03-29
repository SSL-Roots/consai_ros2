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


#include <algorithm>

#include "consai_robot_controller/tactic/tactic_obstacle_avoidance.hpp"
#include "consai_robot_controller/tools/geometry_tools.hpp"

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

bool ObstacleAvoidance::avoid_placement_area(
  const TrackedRobot & my_robot, const State & goal_pose, const TrackedBall & ball,
  const bool avoid_kick_receive_area,
  const State & designated_position, State & avoidance_pose) const
{
  // プレースメント範囲を回避する
  const double THRESHOLD_Y = 1.0;
  const double THRESHOLD_X = 0.65;
  const double AVOIDANCE_POS_X = 0.9;
  const double AVOIDANCE_POS_Y = 0.9;

  auto my_robot_pose = tools::pose_state(my_robot);
  auto ball_pose = tools::pose_state(ball);
  tools::Trans trans_BtoD(ball_pose, tools::calc_angle(ball_pose, designated_position));

  auto robot_pose_BtoD = trans_BtoD.transform(my_robot_pose);
  auto goal_pose_BtoD = trans_BtoD.transform(goal_pose);
  auto designated_BtoD = trans_BtoD.transform(designated_position);

  // 0.5 m 離れなければならない
  // 現在位置と目標位置がともにプレースメントエリアにある場合、回避点を生成する

  // 自チームのプレースメント時は、ボールを蹴る位置、受け取る位置を避けない
  double threshold_x = 0.0;
  if (avoid_kick_receive_area) {
    threshold_x = THRESHOLD_X;
  }
  bool my_pose_is_in_area = std::fabs(robot_pose_BtoD.y) < THRESHOLD_Y &&
    robot_pose_BtoD.x > -threshold_x &&
    robot_pose_BtoD.x < designated_BtoD.x + threshold_x;
  bool goal_pose_is_in_area = std::fabs(goal_pose_BtoD.y) < THRESHOLD_Y &&
    goal_pose_BtoD.x > -threshold_x &&
    goal_pose_BtoD.x < designated_BtoD.x + threshold_x;

  if (my_pose_is_in_area || goal_pose_is_in_area) {
    auto avoid_y = std::copysign(AVOIDANCE_POS_Y, robot_pose_BtoD.y);
    avoidance_pose = trans_BtoD.inverted_transform(robot_pose_BtoD.x, avoid_y, 0.0);

    // デッドロック回避
    const double FIELD_HALF_X = 6.0;
    const double FIELD_HALF_Y = 4.5;
    const double FIELD_WALL_X = 6.3;
    const double FIELD_WALL_Y = 4.8;

    const double FIELD_NEAR_WALL_X = FIELD_HALF_X - 0.0;
    const double FIELD_NEAR_WALL_Y = FIELD_HALF_Y - 0.0;
    auto avoid_x = std::copysign(AVOIDANCE_POS_X + 0.7, avoidance_pose.x);
    avoid_y = std::copysign(AVOIDANCE_POS_Y + 0.7, avoidance_pose.y);

    // フィールド外の場合，壁沿いに回避位置を生成
    if (FIELD_NEAR_WALL_Y < std::fabs(avoidance_pose.y)) {
      avoidance_pose.x = my_robot_pose.x - avoid_x;
    } else if (FIELD_NEAR_WALL_X < std::fabs(avoidance_pose.x)) {
      avoidance_pose.y = my_robot_pose.y - avoid_y;
    }

    // 壁の外に回避位置がある場合
    if (FIELD_WALL_Y < std::fabs(avoidance_pose.y)) {
      avoidance_pose.x = my_robot_pose.x;
      avoidance_pose.y = my_robot_pose.y - std::copysign(1.0, avoidance_pose.y);
    } else if (FIELD_WALL_X < std::fabs(avoidance_pose.x)) {
      avoidance_pose.x = my_robot_pose.x - std::copysign(1.0, avoidance_pose.x);
      avoidance_pose.y = my_robot_pose.y;
    }

    avoidance_pose.theta = my_robot_pose.theta;
  } else {
    avoidance_pose = goal_pose;
  }

  return true;
}

bool ObstacleAvoidance::avoid_robots(
  const TrackedRobot & my_robot, const State & goal_pose,
  State & avoidance_pose) const
{
  // ロボットを回避するposeを生成する
  // 全ロボット情報を検索し、
  // 目標位置とロボットが重なっている場合は、
  // 自己位置方向に目標位置をずらす

  const double ROBOT_DIAMETER = 0.18;  // meters ロボットの直径

  // ロボットを全探索
  for (const auto & robot : detection_->extract_robots()) {
    // 自身の情報は除外する
    if (robot.robot_id.id == my_robot.robot_id.id &&
      robot.robot_id.team_color == my_robot.robot_id.team_color)
    {
      continue;
    }

    // ロボットの位置が目標位置上に存在するか判定
    auto robot_pose = tools::pose_state(robot);
    auto distance = tools::distance(robot_pose, goal_pose);
    if (distance < ROBOT_DIAMETER) {
      // 自己方向にずらした目標位置を生成
      auto my_robot_pose = tools::pose_state(my_robot);
      tools::Trans trans_GtoM(goal_pose, tools::calc_angle(goal_pose, my_robot_pose));
      avoidance_pose = trans_GtoM.inverted_transform(ROBOT_DIAMETER, 0.0, 0.0);
      avoidance_pose.theta = goal_pose.theta;
      return true;
    }
  }

  // 障害物がなければ、目標位置を回避位置とする
  avoidance_pose = goal_pose;
  return true;
}

bool ObstacleAvoidance::avoid_ball_500mm(
  const TrackedRobot & my_robot,
  const State & final_goal_pose,
  const State & goal_pose, const TrackedBall & ball,
  State & avoidance_pose) const
{
  // ボールから500 mm以上離れるために、回避処理を実行する
  // 目標位置がボールに近い場合はボールと目標位置の直線上で位置を離す
  // 回避後の目標位置がフィールド白線外部に生成された場合は、ボールの回避円周上で目標位置をずらす
  const double DISTANCE_TO_AVOID_THRESHOLD = 0.60;
  const double AVOID_MARGIN = 0.09;
  const double DISTANCE_TO_AVOID = DISTANCE_TO_AVOID_THRESHOLD + AVOID_MARGIN;

  const auto robot_pose = tools::pose_state(my_robot);
  const auto ball_pose = tools::pose_state(ball);

  auto avoidance_on_line_robot_to_goal = [&]() {
      // 自分と目標位置を結ぶ座標系を生成
      const auto trans_RtoG = tools::Trans(robot_pose, tools::calc_angle(robot_pose, goal_pose));
      const auto ball_pose_RtoG = trans_RtoG.transform(ball_pose);
      const auto goal_pose_RtoG = trans_RtoG.transform(goal_pose);

      // 自分と目標位置間にボールがなければ終了
      if (ball_pose_RtoG.x < 0.0 || ball_pose_RtoG.x > goal_pose_RtoG.x) {
        return true;
      }

      // ボールが離れていれば終了
      if (std::fabs(ball_pose_RtoG.y) > DISTANCE_TO_AVOID_THRESHOLD) {
        return true;
      }

      // 回避位置を生成
      const auto avoid_x = ball_pose_RtoG.x;
      const auto avoid_y = ball_pose_RtoG.y - std::copysign(DISTANCE_TO_AVOID, ball_pose_RtoG.y);
      avoidance_pose = trans_RtoG.inverted_transform(avoid_x, avoid_y, 0.0);
      return true;
    };

  auto make_a_gap_from_ball = [&]() {
      const auto distance_BtoA = tools::distance(ball_pose, avoidance_pose);
      // 目標位置がボールから離れていれば終了
      if (distance_BtoA > DISTANCE_TO_AVOID_THRESHOLD) {
        return true;
      }

      // 目標位置とボールを結ぶ直線上で、目標位置をボールから離す
      // このとき、最終目標位置側に回避位置を置く
      const auto trans_BtoA = tools::Trans(
        ball_pose, tools::calc_angle(ball_pose, avoidance_pose));
      const auto final_goal_pose_BtoA = trans_BtoA.transform(final_goal_pose);

      avoidance_pose = trans_BtoA.inverted_transform(
        std::copysign(DISTANCE_TO_AVOID, final_goal_pose_BtoA.x), 0.0, 0.0);
      return true;
    };

  auto avoid_outside_of_field = [&]() {
      // フィールド外に目標位置が置かれた場合の処理
      // どれだけフィールドからはみ出たかを、0.0 ~ 1.0に変換する
      // はみ出た分だけ目標位置をボール周囲でずらす
      const auto trans_BtoA = tools::Trans(
        ball_pose, tools::calc_angle(ball_pose, avoidance_pose));
      const auto gain_x =
        std::clamp(
        (std::fabs(avoidance_pose.x) - field_half_length_) / field_boundary_width_, 0.0, 1.0);
      const auto gain_y =
        std::clamp(
        (std::fabs(avoidance_pose.y) - field_half_width_) / field_boundary_width_, 0.0, 1.0);

      if (gain_x > 0.0) {
        auto add_angle = std::copysign(gain_x * M_PI * 0.5, avoidance_pose.y);
        avoidance_pose = trans_BtoA.inverted_transform(
          DISTANCE_TO_AVOID * std::cos(add_angle),
          DISTANCE_TO_AVOID * std::sin(add_angle), 0.0);
      }
      if (gain_y > 0.0) {
        auto add_angle = std::copysign(gain_y * M_PI * 0.5, avoidance_pose.x);
        avoidance_pose = trans_BtoA.inverted_transform(
          DISTANCE_TO_AVOID * std::cos(add_angle),
          DISTANCE_TO_AVOID * std::sin(add_angle), 0.0);
      }
      return true;
    };

  // 障害物がなければ、目標位置を回避位置とする
  avoidance_pose = goal_pose;

  avoidance_on_line_robot_to_goal();
  make_a_gap_from_ball();
  avoid_outside_of_field();

  avoidance_pose.theta = final_goal_pose.theta;

  return true;
}

}  // namespace tactic
