// Copyright 2021 Roots
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

#include <memory>

#include "consai_robot_controller/field_info_parser.hpp"

namespace consai_robot_controller
{

FieldInfoParser::FieldInfoParser(
  const bool team_is_yellow, const bool invert,
  const std::shared_ptr<parser::DetectionExtractor> & detection_extractor)
: team_is_yellow_(team_is_yellow), invert_(invert), detection_extractor_(detection_extractor)
{
  constraint_parser_ = std::make_shared<parser::ConstraintParser>(
    detection_extractor_, team_is_yellow_);
  tactic_control_ball_ = std::make_shared<tactic::ControlBall>();
  tactic_obstacle_avoidance_ = std::make_shared<tactic::ObstacleAvoidance>(detection_extractor);
}

void FieldInfoParser::set_detection_tracked(const TrackedFrame::SharedPtr detection_tracked)
{
  detection_extractor_->set_detection_tracked(detection_tracked);
}

void FieldInfoParser::set_geometry(const GeometryData::SharedPtr geometry)
{
}

void FieldInfoParser::set_referee(const Referee::SharedPtr referee)
{
  referee_ = referee;
}

void FieldInfoParser::set_parsed_referee(const ParsedReferee::SharedPtr parsed_referee)
{
  parsed_referee_ = parsed_referee;
}

void FieldInfoParser::set_named_targets(const NamedTargets::SharedPtr msg)
{
  constraint_parser_->set_named_targets(msg);
}

bool FieldInfoParser::is_parsable(
  const std::shared_ptr<const RobotControl::Goal> goal) const
{
  State target_pose;
  return parse_constraints(goal, target_pose);
}

bool FieldInfoParser::parse_goal(
  const std::shared_ptr<const RobotControl::Goal> goal,
  const TrackedRobot & my_robot, State & parsed_pose, State & final_goal_pose,
  double & kick_power, double & dribble_power)
{
  // RobotControlのgoalを解析し、目標姿勢を出力する
  // 解析に失敗したらfalseを返す
  // 衝突回避やキック、レシーブの処理も実施する

  // 目標姿勢を算出
  if (!parse_constraints(goal, parsed_pose)) {
    return false;
  }

  // ボール受取や、衝突回避に影響されない、最終目標姿勢を格納する
  final_goal_pose = parsed_pose;

  // 以下、ボールが関わる処理のためボール情報を取得する
  TrackedBall ball;
  if (!detection_extractor_->extract_ball(ball)) {
    // ボール情報を取得できなくても正常終了
    return true;
  }

  State kick_target;
  State dribble_target;
  const auto has_kick_target = constraint_parser_->parse_constraint_xy(
    goal->kick_target, kick_target.x, kick_target.y);
  const auto has_dribble_target = constraint_parser_->parse_constraint_xy(
    goal->dribble_target, dribble_target.x, dribble_target.y);

  if (goal->reflect_shoot && has_kick_target) {
    // ボールを受け取りながら目標へ向かって蹴るリフレクトシュート
    if (tactic_control_ball_->reflect_kick(
        kick_target, my_robot, ball, goal->kick_pass, parsed_pose, kick_power, dribble_power))
    {
      return true;
    }
  }

  if (goal->receive_ball) {
    // 転がっているボールを受け取る
    if (tactic_control_ball_->receive_ball(my_robot, ball, parsed_pose, dribble_power)) {
      return true;
    }
  }

  if (goal->kick_enable && has_kick_target) {
    if (goal->kick_setplay) {
      if (tactic_control_ball_->kick_ball_at_setplay(
          kick_target, my_robot, ball, goal->kick_pass, parsed_pose, kick_power, dribble_power))
      {
        return true;
      }
    }

    if (tactic_control_ball_->kick_ball(
        kick_target, my_robot, ball, goal->kick_pass, parsed_pose, kick_power, dribble_power))
    {
      return true;
    }
  }

  if (goal->dribble_enable && has_dribble_target) {
    if (tactic_control_ball_->dribble_ball(
        dribble_target, my_robot, ball, parsed_pose, dribble_power))
    {
      return true;
    }
  }

  if (goal->ball_boy_dribble_enable && has_dribble_target) {
    ball_boy_tactics_.update(dribble_target, my_robot, ball, parsed_pose, dribble_power);
  }

  return true;
}

State FieldInfoParser::modify_goal_pose_to_avoid_obstacles(
  const std::shared_ptr<const RobotControl::Goal> goal,
  const TrackedRobot & my_robot,
  const State & goal_pose, const State & final_goal_pose) const
{
  State avoidance_pose = goal_pose;

  if (!goal->avoid_obstacles) {
    return avoidance_pose;
  }

  TrackedBall ball;
  if (!detection_extractor_->extract_ball(ball)) {
    return avoidance_pose;
  }

  bool avoid_ball = false;
  // STOP_GAME中はボールから離れる
  // TODO(ShotaAk): ここは戦略側から指示をもらうべし
  if (parsed_referee_) {
    if (parsed_referee_->is_our_setplay == false && parsed_referee_->is_inplay == false) {
      avoid_ball = true;
    }
  }

  tactic_obstacle_avoidance_->avoid_obstacles(
    my_robot, goal_pose, ball, avoid_ball, avoidance_pose);

  if (!referee_) {
    return avoidance_pose;
  }

  // ボールプレイスメントエリアを回避する
  if (goal->avoid_placement_area) {
    if (referee_->designated_position.size() > 0) {
      State designated_position;
      designated_position.x = referee_->designated_position[0].x * 0.001;  // mm to meters
      designated_position.y = referee_->designated_position[0].y * 0.001;  // mm to meters

      // サイド反転
      if (invert_) {
        designated_position.x *= -1.0;
        designated_position.y *= -1.0;
      }

      bool is_our_placement =
        (referee_->command == Referee::COMMAND_BALL_PLACEMENT_YELLOW &&
        team_is_yellow_ == true) ||
        (referee_->command == Referee::COMMAND_BALL_PLACEMENT_BLUE && team_is_yellow_ == false);
      bool avoid_kick_receive_area = true;
      // 自チームのプレースメント時は、キック、レシーブエリアを避けない
      if (is_our_placement) {
        avoid_kick_receive_area = false;
      }

      tactic_obstacle_avoidance_->avoid_placement_area(
        my_robot, avoidance_pose, ball, avoid_kick_receive_area,
        designated_position, avoidance_pose);
    }
  }

  // STOP中、プレースメント中は目標位置とロボットの重なりを回避する
  if (referee_->command == Referee::COMMAND_BALL_PLACEMENT_YELLOW ||
    referee_->command == Referee::COMMAND_BALL_PLACEMENT_BLUE ||
    referee_->command == Referee::COMMAND_STOP)
  {
    tactic_obstacle_avoidance_->avoid_robots(my_robot, avoidance_pose, avoidance_pose);
  }

  // STOP_GAME中はボールから離れる
  if (avoid_ball) {
    tactic_obstacle_avoidance_->avoid_ball_500mm(
      my_robot, final_goal_pose, avoidance_pose, ball, avoidance_pose);
  }

  return avoidance_pose;
}

bool FieldInfoParser::parse_constraints(
  const std::shared_ptr<const RobotControl::Goal> goal, State & parsed_pose) const
{
  if (goal->pose.size() > 0) {
    if (constraint_parser_->parse_constraint_pose(goal->pose[0], parsed_pose)) {
      return true;
    }
  }
  if (goal->line.size() > 0) {
    if (constraint_parser_->parse_constraint_line(goal->line[0], parsed_pose)) {
      return true;
    }
  }
  return false;
}


}  // namespace consai_robot_controller
