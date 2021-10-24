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


#include "consai_robot_controller/field_info_parser.hpp"
#include "robocup_ssl_msgs/msg/robot_id.hpp"

namespace consai_robot_controller
{

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

bool FieldInfoParser::extract_robot(const unsigned int robot_id, const bool team_is_yellow, TrackedRobot & my_robot) const
{
  // detection_trackedから指定された色とIDのロボット情報を抽出する
  // visibilityが低いときは情報が無いと判定する
  const double VISIBILITY_THRESHOLD = 0.01;
  for(auto robot : detection_tracked_->robots){
    if(robot_id != robot.robot_id.id){
      continue;
    }
    bool is_yellow = team_is_yellow && robot.robot_id.team_color == RobotId::TEAM_COLOR_YELLOW;
    bool is_blue = !team_is_yellow && robot.robot_id.team_color == RobotId::TEAM_COLOR_BLUE;
    if(!is_yellow && !is_blue){
      continue;
    }
    // if((team_is_yellow && robot.robot_id.team_color != RobotId::TEAM_COLOR_YELLOW) &&
    //    (!team_is_yellow && robot.robot_id.team_color != RobotId::TEAM_COLOR_BLUE)){
    //   continue;
    // }
    if(robot.visibility.size() == 0){
      return false;
    }
    if(robot.visibility[0] < VISIBILITY_THRESHOLD){
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
  for(auto ball : detection_tracked_->balls){
    if(ball.visibility.size() == 0){
      return false;
    }
    if(ball.visibility[0] < VISIBILITY_THRESHOLD){
      return false;
    }

    my_ball = ball;
    break;
  }
  return true;
}

bool FieldInfoParser::parse_goal(const std::shared_ptr<const RobotControl::Goal> goal, State & parsed_pose) const
{
  // RobotControlのgoalを解析し、目標姿勢を出力する
  // 解析に失敗したらfalseを返す
  if(!parse_constraint(goal->x, parsed_pose, parsed_pose.x)){
    return false;
  }
  if(!parse_constraint(goal->y, parsed_pose, parsed_pose.y)){
    return false;
  }
  // 正規化、goal->x.value、goal->y.valueに-1.0 ~ 1.0が入力されている前提
  if(goal->normalize_xy){
    parsed_pose.x *= geometry_->field.field_length * 0.5 * 0.001;  // mm to meters
    parsed_pose.y *= geometry_->field.field_width * 0.5 * 0.001;  // mm to meters
  }

  // オフセットを加算
  parsed_pose.x += goal->offset_x;
  parsed_pose.y += goal->offset_y;

  if(!parse_constraint(goal->theta, parsed_pose, parsed_pose.theta)){
    return false;
  }

  parsed_pose.theta = normalize_theta(parsed_pose.theta + goal->offset_theta);
  return true;
}

bool FieldInfoParser::parse_constraint(const ConstraintTarget & target, const State & current_goal_pose, double & parsed_value) const
{
  // ConstraintTargetを解析する
  // ボールやロボットが存在しない等で値が得られなければfalseを返す
  bool retval = false;
  
  // 実数値
  if(target.value.size() > 0){
    parsed_value = target.value[0];
    retval = true;
  }

  State object_pose;
  bool object_is_exist = false;
  // ボールパラメータ
  if(target.target.size() > 0 && target.target_parameter.size() > 0){
    TrackedBall ball;
    if(target.target[0] == ConstraintTarget::TARGET_BALL && extract_ball(ball)){
      object_pose.x = ball.pos.x;
      object_pose.y = ball.pos.y;
      object_is_exist = true;
    }
  }

  // ロボットパラメータ
  if(target.target_id.size() > 0 && target.target.size() > 0 && target.target_parameter.size() > 0){
    TrackedRobot robot;
    if((target.target[0] == ConstraintTarget::TARGET_BLUE_ROBOT && extract_robot(target.target_id[0], false, robot)) || 
       (target.target[0] == ConstraintTarget::TARGET_YELLOW_ROBOT && extract_robot(target.target_id[0], true, robot))){
      object_pose.x = robot.pos.x;
      object_pose.y = robot.pos.y;
      object_pose.theta = robot.orientation;
      object_is_exist = true;
    }
  }

  if(object_is_exist){
    if(target.target_parameter[0] == ConstraintTarget::PARAMETER_X){
      parsed_value = object_pose.x;
      retval = true;
    }else if(target.target_parameter[0] == ConstraintTarget::PARAMETER_Y){
      parsed_value = object_pose.y;
      retval = true;
    }else if(target.target_parameter[0] == ConstraintTarget::PARAMETER_THETA){
      parsed_value = object_pose.theta;
      retval = true;
    }else if(target.target_parameter[0] == ConstraintTarget::PARAMETER_LOOK_TO){
      parsed_value = calc_angle(current_goal_pose, object_pose);
      retval = true;
    }else if(target.target_parameter[0] == ConstraintTarget::PARAMETER_LOOK_FROM){
      parsed_value = calc_angle(object_pose, current_goal_pose);
      retval = true;
    }
  }

  return retval;
}

double FieldInfoParser::calc_angle(const State & from_pose, const State & to_pose) const
{
  // from_poseからto_poseへの角度を計算する
  double diff_x = to_pose.x - from_pose.x;
  double diff_y = to_pose.y - from_pose.y;

  return std::atan2(diff_y, diff_x);
}

double FieldInfoParser::normalize_theta(const double theta) const
{
  // 角度を-pi ~ piに納める
  double retval = theta;
  while(retval >= M_PI) retval -= 2.0 * M_PI;
  while(retval <= -M_PI) retval += 2.0 * M_PI;
  return retval;
}

}  // namespace consai_robot_controller
