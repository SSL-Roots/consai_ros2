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

#include <algorithm>
#include <memory>
#include <vector>

#include "consai_robot_controller/field_info_parser.hpp"
#include "consai_robot_controller/geometry_tools.hpp"
#include "consai_robot_controller/obstacle_ball.hpp"
#include "consai_robot_controller/obstacle_environment.hpp"
#include "consai_robot_controller/obstacle_robot.hpp"
#include "consai_robot_controller/obstacle_typedef.hpp"
#include "consai_robot_controller/prohibited_area.hpp"
#include "robocup_ssl_msgs/msg/robot_id.hpp"

namespace consai_robot_controller
{

using ObstArea = obstacle::ProhibitedArea;
using ObstBall = obstacle::ObstacleBall;
using ObstEnv = obstacle::ObstacleEnvironment;
using ObstPos = obstacle::Position;
using ObstRadius = obstacle::Radius;
using ObstRobot = obstacle::ObstacleRobot;
using RobotId = robocup_ssl_msgs::msg::RobotId;
namespace tools = geometry_tools;
const double VISIBILITY_THRESHOLD = 0.01;
const double MAX_KICK_POWER_SHOOT = 5.5;  // m/s
const double MAX_KICK_POWER_PASS = 4.0;  // m/s
const double MIN_KICK_POWER_PASS = 2.0;  // m/s

FieldInfoParser::FieldInfoParser(const bool team_is_yellow, const bool invert,
  const std::shared_ptr<parser::DetectionExtractor> & detection_extractor)
: team_is_yellow_(team_is_yellow), invert_(invert), detection_extractor_(detection_extractor)
{
  // 不正な値を参照しないように、フィールド情報の初期値をセットする
  geometry_ = std::make_shared<GeometryData>();
  geometry_->field.field_length = 12000;
  geometry_->field.field_width = 9000;
  geometry_->field.goal_width = 1800;
  geometry_->field.goal_depth = 180;
  geometry_->field.boundary_width = 300;

  constraint_parser_ = std::make_shared<parser::ConstraintParser>(
    detection_extractor_, team_is_yellow_);
  tactic_control_ball_ = std::make_shared<tactic::ControlBall>();
  tactic_obstacle_avoidance_ = std::make_shared<tactic::ObstacleAvoidance>(detection_extractor);
}

void FieldInfoParser::set_detection_tracked(const TrackedFrame::SharedPtr detection_tracked)
{
  detection_tracked_ = detection_tracked;
  detection_extractor_->set_detection_tracked(detection_tracked);
}

void FieldInfoParser::set_geometry(const GeometryData::SharedPtr geometry)
{
  geometry_ = geometry;
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

  // トピックを受け取るたびに初期化する
  named_targets_.clear();

  for (std::size_t i = 0; i < msg->name.size(); ++i) {
    auto name = msg->name[i];
    auto pose = msg->pose[i];
    named_targets_[name] = pose;
  }
}

bool FieldInfoParser::parse_goal(
  const std::shared_ptr<const RobotControl::Goal> goal,
  State & parsed_pose) const
{
  // RobotControlのgoalを解析し、目標姿勢を出力する
  // 解析に失敗したらfalseを返す
  // ここではキック処理や、レシーブ処理をしない

  // 目標姿勢を算出
  State target_pose;
  bool parse_succeeded = false;
  if (goal->pose.size() > 0) {
    if (constraint_parser_->parse_constraint_pose(goal->pose[0], target_pose)) {
      parse_succeeded = true;
    }
  }
  if (goal->line.size() > 0) {
    if (constraint_parser_->parse_constraint_line(goal->line[0], target_pose)) {
      parse_succeeded = true;
    }
  }

  if (parse_succeeded == false) {
    return false;
  }
  parsed_pose = target_pose;
  return true;
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
  if (!parse_goal(goal, parsed_pose)) {
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

  State target;
  bool result = false;
  if (goal->reflect_shoot &&
    constraint_parser_->parse_constraint_xy(goal->kick_target, target.x, target.y) ) {
    // ボールを受け取りながら目標へ向かって蹴るリフレクトシュート
    result = tactic_control_ball_->reflect_kick(
      target, my_robot, ball, goal->kick_pass, parsed_pose, kick_power,
      dribble_power);
  }

  if (goal->receive_ball && result == false) {
    // 転がっているボールを受け取る
    result = tactic_control_ball_->receive_ball(my_robot, ball, parsed_pose, dribble_power);
  }

  if (tools::distance(tools::pose_state(ball), parsed_pose) < 0.7 && result == false) {
    // 目標姿勢とボールが近ければ、ボールを操作する
    if (goal->kick_enable &&
      constraint_parser_->parse_constraint_xy(goal->kick_target, target.x, target.y))
    {
      parse_kick(
        target, my_robot, ball, goal->kick_pass, goal->kick_setplay, parsed_pose,
        kick_power, dribble_power);
    } else if (goal->dribble_enable &&  // NOLINT
      constraint_parser_->parse_constraint_xy(goal->dribble_target, target.x, target.y))
    {
      parse_dribble(target, my_robot, ball, parsed_pose, dribble_power);
    }
  }

  if (goal->ball_boy_dribble_enable &&  // NOLINT
    constraint_parser_->parse_constraint_xy(goal->dribble_target, target.x, target.y)
    && result == false)
  {
    ball_boy_tactics_.update(target, my_robot, ball, parsed_pose, dribble_power);
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
  if (parsed_referee_->is_our_setplay == false && parsed_referee_->is_inplay == false) {
    avoid_ball = true;
  }
  tactic_obstacle_avoidance_->avoid_obstacles(
    my_robot, goal_pose, ball, avoid_ball, avoidance_pose);

  // ボールプレイスメントエリアを回避する
  if (referee_->command == Referee::COMMAND_BALL_PLACEMENT_YELLOW ||
    referee_->command == Referee::COMMAND_BALL_PLACEMENT_BLUE)
  {
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
      if (goal->avoid_placement) {
        tactic_obstacle_avoidance_->avoid_placement_area(
          my_robot, avoidance_pose, ball, avoid_kick_receive_area,
          designated_position, avoidance_pose);
      }
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

bool FieldInfoParser::parse_kick(
  const State & kick_target, const TrackedRobot & my_robot, const TrackedBall & ball,
  const bool & kick_pass, const bool & kick_setplay,
  State & parsed_pose, double & parsed_kick_power, double & parsed_dribble_power) const
{
  const double DRIBBLE_DISTANCE = 0.0;
  const double DRIBBLE_POWER = 1.0;
  bool need_kick = false;
  bool need_dribble = false;

  auto robot_pose = tools::pose_state(my_robot);
  double distance = tools::distance(robot_pose, kick_target);

  // パス速度を設定
  double kick_power_pass = MAX_KICK_POWER_PASS;
  if (distance < 1.0) {
    kick_power_pass = MIN_KICK_POWER_PASS;
  } else {
    // 2.0m離れてたら 2.0 m/sでける
    kick_power_pass = std::min(distance, MAX_KICK_POWER_PASS);
  }

  if (kick_setplay) {
    tactic_control_ball_->control_ball_at_setplay(
      kick_target, my_robot, ball, parsed_pose, need_kick, need_dribble);
  } else {
    tactic_control_ball_->control_ball(
      kick_target, my_robot, ball, DRIBBLE_DISTANCE, parsed_pose, need_kick, need_dribble);
  }

  if (need_dribble) {
    parsed_dribble_power = DRIBBLE_POWER;
  } else {
    parsed_dribble_power = 0.0;
  }

  if (need_kick) {
    if (kick_pass) {
      parsed_kick_power = kick_power_pass;
    } else {
      parsed_kick_power = MAX_KICK_POWER_SHOOT;
    }
  } else {
    parsed_kick_power = 0.0;
  }
  return true;
}

bool FieldInfoParser::parse_dribble(
  const State & dribble_target, const TrackedRobot & my_robot, const TrackedBall & ball,
  State & parsed_pose, double & parsed_dribble_power) const
{
  const double DRIBBLE_DISTANCE = 0.15;
  const double DRIBBLE_POWER = 1.0;
  bool need_kick = false;
  bool need_dribble = false;

  tactic_control_ball_->control_ball(
    dribble_target, my_robot, ball, DRIBBLE_DISTANCE, parsed_pose, need_kick, need_dribble);

  if (need_dribble) {
    parsed_dribble_power = DRIBBLE_POWER;
  } else {
    parsed_dribble_power = 0.0;
  }
  return true;
}


}  // namespace consai_robot_controller
