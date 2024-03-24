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
    if (parse_constraint_pose(goal->pose[0], target_pose)) {
      parse_succeeded = true;
    }
  }
  if (goal->line.size() > 0) {
    if (parse_constraint_line(goal->line[0], target_pose)) {
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
  if (goal->reflect_shoot && parse_constraint_xy(goal->kick_target, target.x, target.y) ) {
    // ボールを受け取りながら目標へ向かって蹴るリフレクトシュート
    result = reflect_kick(
      target, my_robot, ball, goal->kick_pass, parsed_pose, kick_power,
      dribble_power);
  }

  if (goal->receive_ball && result == false) {
    // 転がっているボールを受け取る
    result = receive_ball(my_robot, ball, parsed_pose, dribble_power);
  }

  if (tools::distance(tools::pose_state(ball), parsed_pose) < 0.7 && result == false) {
    // 目標姿勢とボールが近ければ、ボールを操作する
    if (goal->kick_enable &&
      parse_constraint_xy(goal->kick_target, target.x, target.y))
    {
      parse_kick(
        target, my_robot, ball, goal->kick_pass, goal->kick_setplay, parsed_pose,
        kick_power, dribble_power);
    } else if (goal->dribble_enable &&  // NOLINT
      parse_constraint_xy(goal->dribble_target, target.x, target.y))
    {
      parse_dribble(target, my_robot, ball, parsed_pose, dribble_power);
    }
  }

  if (goal->ball_boy_dribble_enable &&  // NOLINT
    parse_constraint_xy(goal->dribble_target, target.x, target.y) && result == false)
  {
    ball_boy_tactics_.update(target, my_robot, ball, parsed_pose, dribble_power);
  }

  return true;
}

std::vector<unsigned int> FieldInfoParser::active_robot_id_list(const bool team_is_yellow) const
{
  // 存在しているロボットのIDリストを返す
  std::vector<unsigned int> id_list;
  if (detection_tracked_) {
    for (const auto & robot : detection_tracked_->robots) {
      bool is_yellow = team_is_yellow && robot.robot_id.team_color == RobotId::TEAM_COLOR_YELLOW;
      bool is_blue = !team_is_yellow && robot.robot_id.team_color == RobotId::TEAM_COLOR_BLUE;
      if (!is_yellow && !is_blue) {
        continue;
      }

      if (robot.visibility.size() == 0) {
        continue;
      }
      if (robot.visibility[0] < VISIBILITY_THRESHOLD) {
        continue;
      }

      id_list.push_back(robot.robot_id.id);
    }
  }
  return id_list;
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
  avoid_obstacles(my_robot, goal_pose, ball, avoid_ball, avoidance_pose);

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
        avoid_placement_area(
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
    avoid_robots(my_robot, avoidance_pose, avoidance_pose);
  }

  // STOP_GAME中はボールから離れる
  if (avoid_ball) {
    avoid_ball_500mm(my_robot, final_goal_pose, avoidance_pose, ball, avoidance_pose);
  }

  return avoidance_pose;
}

obstacle::ObstacleEnvironment FieldInfoParser::get_obstacle_environment(
  const std::shared_ptr<const RobotControl::Goal> goal,
  const TrackedRobot & my_robot) const
{
  constexpr ObstRadius BALL_RADIUS(0.0215);
  constexpr ObstRadius ROBOT_RADIUS(0.09);

  // goalから障害物環境を作成する
  ObstEnv environment;

  // 衝突回避しない場合は空の環境を返す
  if (!goal->avoid_obstacles) {
    return environment;
  }

  // ロボット、ボール情報がない場合は空の環境を返す
  if (!detection_tracked_) {
    return environment;
  }

  // インプレイと自チームセットプレイ以外ではボールから離れる
  // TODO(ShotaAk): ここは戦略側で判断できそう
  if (parsed_referee_) {
    if (parsed_referee_->is_our_setplay == false && parsed_referee_->is_inplay == false) {
      TrackedBall ball;
      if (detection_extractor_->extract_ball(ball)) {
        environment.append_obstacle_ball(ObstBall(ObstPos(ball.pos.x, ball.pos.y), BALL_RADIUS));
      }
    }
  }

  // ロボットを障害物として扱う
  for (const auto & robot : detection_tracked_->robots) {
    if (robot.visibility.size() == 0) {
      continue;
    }
    if (robot.visibility[0] < VISIBILITY_THRESHOLD) {
      continue;
    }

    // 自身の情報は除外する
    if (robot.robot_id.id == my_robot.robot_id.id &&
      robot.robot_id.team_color == my_robot.robot_id.team_color)
    {
      continue;
    }

    environment.append_obstacle_robot(
      ObstRobot(ObstPos(robot.pos.x, robot.pos.y), ROBOT_RADIUS));
  }

  // TODO(ShotaAk): ボールプレースメント回避エリアとディフェンスエリアを障害物に追加する

  return environment;
}

bool FieldInfoParser::parse_constraint_pose(const ConstraintPose & pose, State & parsed_pose) const
{
  return constraint_parser_->parse_constraint_pose(pose, parsed_pose);
}

bool FieldInfoParser::parse_constraint_line(
  const ConstraintLine & line, State & parsed_pose) const
{
  return constraint_parser_->parse_constraint_line(line, parsed_pose);
}

bool FieldInfoParser::parse_constraint_xy(
  const ConstraintXY & xy, double & parsed_x,
  double & parsed_y) const
{
  return constraint_parser_->parse_constraint_xy(xy, parsed_x, parsed_y);
}

bool FieldInfoParser::parse_constraint_theta(
  const ConstraintTheta & theta, const double goal_x,
  const double goal_y, double & parsed_theta) const
{
  return constraint_parser_->parse_constraint_theta(theta, goal_x, goal_y, parsed_theta);
}

bool FieldInfoParser::parse_constraint_object(
  const ConstraintObject & object,
  State & object_pose) const
{
  return constraint_parser_->parse_constraint_object(object, object_pose);
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
    control_ball_at_setplay(kick_target, my_robot, ball, parsed_pose, need_kick, need_dribble);
  } else {
    control_ball(
      kick_target, my_robot, ball, DRIBBLE_DISTANCE, parsed_pose, need_kick,
      need_dribble);
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

  control_ball(
    dribble_target, my_robot, ball, DRIBBLE_DISTANCE, parsed_pose, need_kick,
    need_dribble);

  if (need_dribble) {
    parsed_dribble_power = DRIBBLE_POWER;
  } else {
    parsed_dribble_power = 0.0;
  }
  return true;
}

bool FieldInfoParser::parse_ball_boy_dribble(
  const State & dribble_target, const TrackedRobot & my_robot, const TrackedBall & ball,
  State & parsed_pose, double & parsed_dribble_power) const
{
  // ボールボーイロボ用のドリブル関数
  const double BALL_ARRIVAL_DISTANCE = 0.1;
  const double RELEASE_DISTANCE = 0.5;
  const double CATCH_DISTANCE = 0.2;

  const double DRIBBLE_CATCH_POWER = 1.0;
  const double DRIBBLE_RELEASE_POWER = 0.0;
  const double ROBOT_RADIUS = 0.180 * 0.5;
  const auto center_pose = State();
  const auto ball_pose = tools::pose_state(ball);
  const auto robot_pose = tools::pose_state(my_robot);

  // ボールが目標位置に到着したら、ボールから離れた位置に移動する
  if (tools::distance(ball_pose, dribble_target) < BALL_ARRIVAL_DISTANCE) {
    const auto angle_target_to_robot = tools::calc_angle(dribble_target, robot_pose);
    tools::Trans trans_TtoR(dribble_target, angle_target_to_robot);
    parsed_pose = trans_TtoR.inverted_transform(RELEASE_DISTANCE, 0.0, -M_PI);
    parsed_dribble_power = DRIBBLE_RELEASE_POWER;
    return true;
  }

  // ロボットがボールを見ていたら、目標位置に向かって進む
  const auto angle_robot_to_ball = tools::calc_angle(robot_pose, ball_pose);
  const tools::Trans trans_RtoB(robot_pose, angle_robot_to_ball);
  const auto robot_pose_RtoB = trans_RtoB.transform(robot_pose);

  // 早めにキャッチャーを動かす
  std::cout << "theta:" << tools::to_degrees(std::fabs(robot_pose_RtoB.theta)) << std::endl;
  std::cout << "distance:" << tools::distance(robot_pose, ball_pose) << std::endl;

  const auto distance_RtoB = tools::distance(robot_pose, ball_pose);

  // if (std::fabs(robot_pose_RtoB.theta) < tools::to_radians(10) &&
  //     distance_RtoB > ROBOT_RADIUS && distance_RtoB < CATCH_DISTANCE) {
  //   std::cout << "PRE CATCH!!!" << std::endl;
  //   parsed_dribble_power = DRIBBLE_CATCH_POWER;
  // } else {
  //   parsed_dribble_power = DRIBBLE_RELEASE_POWER;
  // }

  if (std::fabs(robot_pose_RtoB.theta) < tools::to_radians(10) &&
    distance_RtoB > ROBOT_RADIUS && distance_RtoB < CATCH_DISTANCE * 1.0)
  {
    parsed_dribble_power = DRIBBLE_CATCH_POWER;

    const auto angle_target_to_robot = tools::calc_angle(dribble_target, robot_pose);
    tools::Trans trans_TtoR(dribble_target, angle_target_to_robot);
    parsed_pose = trans_TtoR.inverted_transform(CATCH_DISTANCE, 0.0, -M_PI);
    return true;
  } else {
    parsed_dribble_power = DRIBBLE_RELEASE_POWER;
  }

  // if (std::fabs(robot_pose_RtoB.theta) < tools::to_radians(10) &&
  //     distance_RtoB > ROBOT_RADIUS && distance_RtoB < CATCH_DISTANCE * 1.2) {
  //   std::cout << "CATCH!!!" << std::endl;
  //   const auto angle_target_to_robot = tools::calc_angle(dribble_target, robot_pose);
  //   tools::Trans trans_TtoR(dribble_target, angle_target_to_robot);
  //   parsed_pose = trans_TtoR.inverted_transform(CATCH_DISTANCE, 0.0, -M_PI);
  //   parsed_dribble_power = DRIBBLE_CATCH_POWER;
  //   return true;
  // }

  // ロボットがボールをキャッチできそうな場合
  const auto angle_ball_to_center = tools::calc_angle(ball_pose, center_pose);
  tools::Trans trans_BtoC(ball_pose, angle_ball_to_center);
  parsed_pose = trans_BtoC.inverted_transform(CATCH_DISTANCE * 0.8, 0.0, -M_PI);

  return true;
}

bool FieldInfoParser::control_ball(
  const State & target, const TrackedRobot & my_robot, const TrackedBall & ball,
  const double & dribble_distance, State & parsed_pose, bool & need_kick, bool & need_dribble) const
{
  return tactic_control_ball_->control_ball(
    target, my_robot, ball, dribble_distance, parsed_pose, need_kick, need_dribble);
}

bool FieldInfoParser::control_ball_at_setplay(
  const State & target, const TrackedRobot & my_robot, const TrackedBall & ball,
  State & parsed_pose, bool & need_kick, bool & need_dribble) const
{
  return tactic_control_ball_->control_ball_at_setplay(
    target, my_robot, ball, parsed_pose, need_kick, need_dribble);
}


bool FieldInfoParser::receive_ball(
  const TrackedRobot & my_robot, const TrackedBall & ball,
  State & parsed_pose, double & parsed_dribble_power) const
{
  return tactic_control_ball_->receive_ball(my_robot, ball, parsed_pose, parsed_dribble_power);
}

bool FieldInfoParser::reflect_kick(
  const State & target, const TrackedRobot & my_robot, const TrackedBall & ball,
  const bool & kick_pass, State & parsed_pose, double & parsed_kick_power,
  double & parsed_dribble_power) const
{
  return tactic_control_ball_->reflect_kick(
    target, my_robot, ball, kick_pass, parsed_pose, parsed_kick_power, parsed_dribble_power);
}

bool FieldInfoParser::avoid_obstacles(
  const TrackedRobot & my_robot, const State & goal_pose, const TrackedBall & ball,
  const bool & avoid_ball, State & avoidance_pose) const
{
  return tactic_obstacle_avoidance_->avoid_obstacles(
    my_robot, goal_pose, ball, avoid_ball, avoidance_pose);
}


bool FieldInfoParser::avoid_placement_area(
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

bool FieldInfoParser::avoid_robots(
  const TrackedRobot & my_robot, const State & goal_pose,
  State & avoidance_pose) const
{
  // ロボットを回避するposeを生成する
  // 全ロボット情報を検索し、
  // 目標位置とロボットが重なっている場合は、
  // 自己位置方向に目標位置をずらす

  const double ROBOT_DIAMETER = 0.18;  // meters ロボットの直径

  // ロボットを全探索
  for (const auto & robot : detection_tracked_->robots) {
    if (robot.visibility.size() == 0) {
      continue;
    }
    if (robot.visibility[0] < VISIBILITY_THRESHOLD) {
      continue;
    }

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

bool FieldInfoParser::avoid_ball_500mm(
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
      if (!geometry_) {
        return true;
      }

      // フィールド外に目標位置が置かれた場合の処理
      const auto BOUNDARY_WIDTH = geometry_->field.boundary_width * 0.001;
      const auto FIELD_HALF_X = geometry_->field.field_length * 0.5 * 0.001;
      const auto FIELD_HALF_Y = geometry_->field.field_width * 0.5 * 0.001;
      // どれだけフィールドからはみ出たかを、0.0 ~ 1.0に変換する
      // はみ出た分だけ目標位置をボール周囲でずらす
      const auto trans_BtoA = tools::Trans(
        ball_pose, tools::calc_angle(ball_pose, avoidance_pose));
      const auto gain_x =
        std::clamp((std::fabs(avoidance_pose.x) - FIELD_HALF_X) / BOUNDARY_WIDTH, 0.0, 1.0);
      const auto gain_y =
        std::clamp((std::fabs(avoidance_pose.y) - FIELD_HALF_Y) / BOUNDARY_WIDTH, 0.0, 1.0);

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

}  // namespace consai_robot_controller
