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
#include "consai_robot_controller/geometry_tools.hpp"
#include "robocup_ssl_msgs/msg/robot_id.hpp"

namespace consai_robot_controller
{

using RobotId = robocup_ssl_msgs::msg::RobotId;
namespace tools = geometry_tools;

FieldInfoParser::FieldInfoParser()
{
}

void FieldInfoParser::set_detection_tracked(const TrackedFrame::SharedPtr detection_tracked)
{
  detection_tracked_ = detection_tracked;
}

void FieldInfoParser::set_geometry(const GeometryData::SharedPtr geometry)
{
  geometry_ = geometry;
}

bool FieldInfoParser::extract_robot(
  const unsigned int robot_id, const bool team_is_yellow,
  TrackedRobot & my_robot) const
{
  // detection_trackedから指定された色とIDのロボット情報を抽出する
  // visibilityが低いときは情報が無いと判定する
  const double VISIBILITY_THRESHOLD = 0.01;
  for (const auto & robot : detection_tracked_->robots) {
    if (robot_id != robot.robot_id.id) {
      continue;
    }
    bool is_yellow = team_is_yellow && robot.robot_id.team_color == RobotId::TEAM_COLOR_YELLOW;
    bool is_blue = !team_is_yellow && robot.robot_id.team_color == RobotId::TEAM_COLOR_BLUE;
    if (!is_yellow && !is_blue) {
      continue;
    }
    // if((team_is_yellow && robot.robot_id.team_color != RobotId::TEAM_COLOR_YELLOW) &&
    //    (!team_is_yellow && robot.robot_id.team_color != RobotId::TEAM_COLOR_BLUE)){
    //   continue;
    // }
    if (robot.visibility.size() == 0) {
      return false;
    }
    if (robot.visibility[0] < VISIBILITY_THRESHOLD) {
      return false;
    }

    my_robot = robot;
    break;
  }
  return true;
}

bool FieldInfoParser::extract_ball(TrackedBall & my_ball) const
{
  // detection_trackedからボール情報を抽出する
  // visibilityが低いときは情報が無いと判定する
  const double VISIBILITY_THRESHOLD = 0.01;
  for (const auto & ball : detection_tracked_->balls) {
    if (ball.visibility.size() == 0) {
      return false;
    }
    if (ball.visibility[0] < VISIBILITY_THRESHOLD) {
      return false;
    }

    my_ball = ball;
    break;
  }
  return true;
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
  const TrackedRobot & my_robot, State & parsed_pose,
  double & kick_power, double & dribble_power) const
{
  // RobotControlのgoalを解析し、目標姿勢を出力する
  // 解析に失敗したらfalseを返す
  // 衝突回避やキック、レシーブの処理も実施する

  // 目標姿勢を算出
  if (!parse_goal(goal, parsed_pose)) {
    return false;
  }

  // 以下、ボールが関わる処理のためボール情報を取得する
  TrackedBall ball;
  if (!extract_ball(ball)) {
    // ボール情報を取得できなくても正常終了
    return true;
  }

  // 転がっているボールを受け取る
  if (goal->receive_ball) {
    receive_ball(my_robot, ball, parsed_pose, dribble_power);
  }

  // ボール蹴る
  State kick_target;
  if (goal->kick_enable &&
    parse_constraint_xy(goal->kick_target, kick_target.x, kick_target.y))
  {
    // 目標姿勢とボールが近ければ、キック処理を行う
    if (tools::distance(tools::pose_state(ball), parsed_pose) < 0.7) {
      parse_kick(kick_target, my_robot, ball, goal->kick_pass, parsed_pose, kick_power, dribble_power);
    }
  }

  // 衝突を回避する
  if (goal->avoid_obstacles) {
    avoid_obstacles(my_robot, parsed_pose, parsed_pose);
  }

  return true;
}

bool FieldInfoParser::parse_constraint_pose(const ConstraintPose & pose, State & parsed_pose) const
{
  double parsed_x, parsed_y;
  if (!parse_constraint_xy(pose.xy, parsed_x, parsed_y)) {
    return false;
  }
  parsed_x += pose.offset.x;
  parsed_y += pose.offset.y;

  double parsed_theta;
  if (!parse_constraint_theta(pose.theta, parsed_x, parsed_y, parsed_theta)) {
    return false;
  }

  parsed_theta = tools::normalize_theta(parsed_theta + pose.offset.theta);

  parsed_pose.x = parsed_x;
  parsed_pose.y = parsed_y;
  parsed_pose.theta = parsed_theta;

  return true;
}

bool FieldInfoParser::parse_constraint_line(
  const ConstraintLine & line, State & parsed_pose) const {
  State p1, p2;
  if (!parse_constraint_xy(line.p1, p1.x, p1.y)) {
    return false;
  }
  if (!parse_constraint_xy(line.p2, p2.x, p2.y)) {
    return false;
  }

  State p3, p4;
  bool has_p3_p4 = false;
  if (line.p3.size() > 0 && line.p4.size()) {
    if (parse_constraint_xy(line.p3[0], p3.x, p3.y) &&
        parse_constraint_xy(line.p4[0], p4.x, p4.y)) {
      has_p3_p4 = true;
    }
  }

  // 直線p1->p2の座標系を作成
  auto angle_p1_to_p2 = tools::calc_angle(p1, p2);
  tools::Trans trans_1to2(p1, angle_p1_to_p2);
  if(has_p3_p4) {
    // p1 ~ p4がセットされていれば、
    // 直線p1->p2上で、直線p3->p4と交わるところを目標位置とする
    State intersection = tools::intersection(p1, p2, p3, p4);
    // 交点が直線p1->p2をはみ出る場合は、p1 or p2に置き換える
    auto intersection_1to2 = trans_1to2.transform(intersection);
    auto p2_1to2 = trans_1to2.transform(p2);
    if (intersection_1to2.x < 0.0) {
      parsed_pose = p1;
    } else if (intersection_1to2.x > p2_1to2.x) {
      parsed_pose = p2;
    } else {
      parsed_pose = intersection;
    }
  }else {
    // 直線p1->p2上で、p1からdistanceだけ離れた位置を目標位置する
    parsed_pose = trans_1to2.inverted_transform(line.distance, 0, 0);
  }

  if (!parse_constraint_theta(line.theta, parsed_pose.x, parsed_pose.y,
                              parsed_pose.theta)) {
    return false;
  }


  return true;
}

bool FieldInfoParser::parse_constraint_xy(
  const ConstraintXY & xy, double & parsed_x,
  double & parsed_y) const
{
  State object_pose;
  if (xy.object.size() > 0) {
    if (parse_constraint_object(xy.object[0], object_pose)) {
      parsed_x = object_pose.x;
      parsed_y = object_pose.y;
      return true;
    }
  }

  if (xy.value_x.size() > 0 && xy.value_y.size() > 0) {
    parsed_x = xy.value_x[0];
    parsed_y = xy.value_y[0];

    // フィールドサイズに対してx, yが-1 ~ 1に正規化されている
    if (xy.normalized) {
      parsed_x *= geometry_->field.field_length * 0.5 * 0.001;
      parsed_y *= geometry_->field.field_width * 0.5 * 0.001;
    }
    return true;
  }

  return false;
}

bool FieldInfoParser::parse_constraint_theta(
  const ConstraintTheta & theta, const double goal_x,
  const double goal_y, double & parsed_theta) const
{
  State object_pose;
  if (theta.object.size() > 0) {
    if (parse_constraint_object(theta.object[0], object_pose)) {
      if (theta.param == ConstraintTheta::PARAM_THETA) {
        parsed_theta = object_pose.theta;
        return true;
      } else if (theta.param == ConstraintTheta::PARAM_LOOK_TO) {
        State goal_pose;
        goal_pose.x = goal_x;
        goal_pose.y = goal_y;
        parsed_theta = tools::calc_angle(goal_pose, object_pose);
        return true;
      } else if (theta.param == ConstraintTheta::PARAM_LOOK_FROM) {
        State goal_pose;
        goal_pose.x = goal_x;
        goal_pose.y = goal_y;
        parsed_theta = tools::calc_angle(object_pose, goal_pose);
        return true;
      }
    }
  }

  if (theta.value_theta.size() > 0) {
    parsed_theta = theta.value_theta[0];
    return true;
  }

  return false;
}

bool FieldInfoParser::parse_constraint_object(
  const ConstraintObject & object,
  State & object_pose) const
{
  TrackedBall ball;
  TrackedRobot robot;

  // NOLINTについて
  // ament_uncrustifyとament_cpplintが競合するので、lintのチェックをスキップする
  // Ref: https://github.com/ament/ament_lint/issues/158
  if (object.type == ConstraintObject::BALL && extract_ball(ball)) {
    object_pose.x = ball.pos.x;
    object_pose.y = ball.pos.y;
    return true;
  } else if ((object.type == ConstraintObject::BLUE_ROBOT &&  // NOLINT
    extract_robot(object.robot_id, false, robot)) ||
    (object.type == ConstraintObject::YELLOW_ROBOT && extract_robot(object.robot_id, true, robot)))
  {
    object_pose.x = robot.pos.x;
    object_pose.y = robot.pos.y;
    object_pose.theta = robot.orientation;
    return true;
  }

  return false;
}

bool FieldInfoParser::parse_kick(
  const State & kick_target, const TrackedRobot & my_robot, const TrackedBall & ball,
  const bool & kick_pass,
  State & parsed_pose, double & parsed_kick_power, double & parsed_dribble_power) const
{
  const double DRIBBLE_POWER = 0.6;
  const double KICK_POWER_SHOOT = 6.5;
  const double KICK_POWER_PASS = 3.0;
  const double LOOKING_BALL_DISTANCE = 0.2;  // meters
  const double LOOKING_BALL_THETA = tools::to_radians(180 - 15);
  const double LOOKING_TARGET_THETA = tools::to_radians(30);
  const double CAN_DRIBBLE_DISTANCE = 0.5;  // meters;
  const double CAN_SHOOT_THETA = tools::to_radians(5);
  const double CAN_PASS_THETA = tools::to_radians(3);
  const double CAN_SHOOT_OMEGA = 0.05;  // rad/s
  const double DISTANCE_TO_LOOK_BALL = -0.1;  // meters
  const double THETA_TO_ROTATE = tools::to_radians(30);  // meters
  const double DISTANCE_TO_KICK_BALL = -0.005;  // meters

  // ボールを向きながらボールに近づく
  auto ball_pose = tools::pose_state(ball);
  auto robot_pose = tools::pose_state(my_robot);

  // ボールからロボットを見た座標系を生成
  auto angle_ball_to_robot = tools::calc_angle(ball_pose, robot_pose);
  tools::Trans trans_BtoR(ball_pose, angle_ball_to_robot);
  auto robot_pose_BtoR = trans_BtoR.transform(robot_pose);

  // ボールからターゲットを見た座標系を生成
  auto angle_ball_to_target = tools::calc_angle(ball_pose, kick_target);
  tools::Trans trans_BtoT(ball_pose, angle_ball_to_target);
  auto robot_pose_BtoT = trans_BtoT.transform(robot_pose);

  bool is_looking_ball = robot_pose_BtoR.x<LOOKING_BALL_DISTANCE &&
      std::fabs(robot_pose_BtoR.theta)> LOOKING_BALL_THETA;

  bool is_looking_target = std::fabs(robot_pose_BtoT.theta) < LOOKING_TARGET_THETA;

  double distance_robot_to_ball = tools::distance(robot_pose, ball_pose);

  // ロボットがボールに近ければドリブルをON
  bool can_dribble = distance_robot_to_ball < CAN_DRIBBLE_DISTANCE;
  // ロボットがキックターゲットに焦点を当てたらキックをON
  bool can_kick = false;
  if (kick_pass) {
    can_kick = std::fabs(robot_pose_BtoT.theta) < CAN_PASS_THETA &&
      std::fabs(my_robot.vel_angular[0]) < CAN_SHOOT_OMEGA;
  } else {
    can_kick = std::fabs(robot_pose_BtoT.theta) < CAN_SHOOT_THETA &&
      std::fabs(my_robot.vel_angular[0]) < CAN_SHOOT_OMEGA;
  }

  if (!is_looking_ball) {
    // ドリブラーがボールに付くまで移動する
    parsed_pose = trans_BtoR.inverted_transform(DISTANCE_TO_LOOK_BALL, 0, M_PI);
    if (can_dribble) {parsed_dribble_power = DRIBBLE_POWER;}
  } else if (!is_looking_target) {
    // キックターゲットを見るまで、ドリブラをボールに付けながら回転する
    double add_angle = -std::copysign(THETA_TO_ROTATE, robot_pose_BtoT.theta);

    parsed_pose = trans_BtoR.inverted_transform(
      distance_robot_to_ball * std::cos(add_angle),
      distance_robot_to_ball * std::sin(add_angle), 0.0);
    parsed_pose.theta = tools::calc_angle(parsed_pose, ball_pose);
    if (can_dribble) {parsed_dribble_power = DRIBBLE_POWER;}
  } else {
    // キックターゲットに向かって前進する
    parsed_pose = trans_BtoT.inverted_transform(DISTANCE_TO_KICK_BALL, 0.0, 0.0);
    if (can_dribble) {parsed_dribble_power = DRIBBLE_POWER;}
    if (can_kick) {
      if (kick_pass) {
        parsed_kick_power = KICK_POWER_PASS;
      } else {
        parsed_kick_power = KICK_POWER_SHOOT;
      }
    }
  }

  return true;
}

bool FieldInfoParser::receive_ball(
  const TrackedRobot & my_robot, const TrackedBall & ball,
  State & parsed_pose, double & parsed_dribble_power) const
{
  // 転がっているボールを受け取る
  const double DRIBBLE_POWER = 0.6;

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
  parsed_dribble_power = DRIBBLE_POWER;

  return true;
}

bool FieldInfoParser::avoid_obstacles(
  const TrackedRobot & my_robot,
  const State & goal_pose, State & avoidance_pose) const
{
  // 障害物を回避するposeを生成する
  // 全ロボット情報を検索し、
  // 自己位置(my_robot)と目標位置(goal_pose)間に存在する、
  // 自己位置に最も近い障害物付近に回避点を生成し、その回避点を新しい目標位置とする

  const double VISIBILITY_THRESHOLD = 0.01;  // 0.0 ~ 1.0
  // 自身から直進方向に何m離れたロボットを障害物と判定するか
  const double OBSTACLE_DETECTION_X = 0.2;
  // 自身から直進方向に対して左右何m離れたロボットを障害物と判定するか
  const double OBSTACLE_DETECTION_Y = 0.5;
  const double AVOIDANCE_POS_X = 0.2;
  const double AVOIDANCE_POS_Y = 0.5;

  auto my_robot_pose = tools::pose_state(my_robot);
  tools::Trans trans_MtoG(my_robot_pose, tools::calc_angle(my_robot_pose, goal_pose));
  auto goal_pose_MtoG = trans_MtoG.transform(goal_pose);

  double distance_to_obstacle = 10000;  // 適当な大きい数字を設定
  std::shared_ptr<State> obstacle_pose_MtoG;

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

    // ロボットの位置が自己位置と目標位置の間に存在するか判定
    auto robot_pose = tools::pose_state(robot);
    auto robot_pose_MtoG = trans_MtoG.transform(robot_pose);

    if (robot_pose_MtoG.x > OBSTACLE_DETECTION_X &&
      robot_pose_MtoG.x < goal_pose_MtoG.x &&
      std::fabs(robot_pose_MtoG.y) < OBSTACLE_DETECTION_Y)
    {
      // 最も自身に近いロボットを障害物とする
      double distance = std::hypot(robot_pose_MtoG.x, robot_pose_MtoG.y);
      if (distance < distance_to_obstacle) {
        obstacle_pose_MtoG = std::make_shared<State>(robot_pose_MtoG);
        distance_to_obstacle = distance;
      }
    }
  }

  // 障害物が存在すれば、回避位置を生成する
  if (obstacle_pose_MtoG) {
    avoidance_pose = trans_MtoG.inverted_transform(
      obstacle_pose_MtoG->x + AVOIDANCE_POS_X,
      obstacle_pose_MtoG->y - std::copysign(AVOIDANCE_POS_Y, obstacle_pose_MtoG->y),
      goal_pose_MtoG.theta
    );
  }

  return true;
}

}  // namespace consai_robot_controller
