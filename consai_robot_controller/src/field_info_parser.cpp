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

  State target_pose;
  if(goal->pose.size() > 0){
    if(parse_constraint_pose(goal->pose[0], target_pose)){
      parsed_pose = target_pose;
      return true;
    }
  }

  return false;
}

bool FieldInfoParser::parse_constraint_pose(const ConstraintPose & pose, State & parsed_pose) const
{
  double parsed_x, parsed_y;
  if(!parse_constraint_xy(pose.xy, parsed_x, parsed_y)){
    return false;
  }
  parsed_x += pose.offset.x;
  parsed_y += pose.offset.y;

  double parsed_theta;
  if(!parse_constraint_theta(pose.theta, parsed_x, parsed_y, parsed_theta)){
    return false;
  }

  parsed_theta = normalize_theta(parsed_theta + pose.offset.theta);

  parsed_pose.x = parsed_x;
  parsed_pose.y = parsed_y;
  parsed_pose.theta = parsed_theta;

  return true;
}

bool FieldInfoParser::parse_constraint_xy(const ConstraintXY & xy, double & parsed_x, double & parsed_y) const
{
  State object_pose;
  if(xy.object.size() > 0){
    if(parse_constraint_object(xy.object[0], object_pose)){
      parsed_x = object_pose.x;
      parsed_y = object_pose.y;
      return true;
    }
  }

  if(xy.value_x.size() > 0 && xy.value_y.size() > 0){
    parsed_x = xy.value_x[0];
    parsed_y = xy.value_y[0];

    // フィールドサイズに対してx, yが-1 ~ 1に正規化されている
    if(xy.normalized){
      parsed_x *= geometry_->field.field_length * 0.5 * 0.001;
      parsed_y *= geometry_->field.field_width * 0.5 * 0.001;
    }
    return true;
  }

  return false;
}

bool FieldInfoParser::parse_constraint_theta(const ConstraintTheta & theta, const double goal_x, const double goal_y, double & parsed_theta) const
{
  State object_pose;
  if(theta.object.size() > 0){
    if(parse_constraint_object(theta.object[0], object_pose)){
      if(theta.param == ConstraintTheta::PARAM_THETA){
        parsed_theta = object_pose.theta;
        return true;
      }else if(theta.param == ConstraintTheta::PARAM_LOOK_TO){
        State goal_pose;
        goal_pose.x = goal_x;
        goal_pose.y = goal_y;
        parsed_theta = calc_angle(goal_pose, object_pose);
        return true;
      }else if(theta.param == ConstraintTheta::PARAM_LOOK_FROM){
        State goal_pose;
        goal_pose.x = goal_x;
        goal_pose.y = goal_y;
        parsed_theta = calc_angle(object_pose, goal_pose);
        return true;
      }
    }
  }

  if(theta.value_theta.size() > 0){
    parsed_theta = theta.value_theta[0];
    return true;
  }

  return false;
}

bool FieldInfoParser::parse_constraint_object(const ConstraintObject & object, State & object_pose) const
{
  TrackedBall ball;
  TrackedRobot robot;

  if(object.type == ConstraintObject::BALL && extract_ball(ball)){
    object_pose.x = ball.pos.x;
    object_pose.y = ball.pos.y;
    return true;
  }else if((object.type == ConstraintObject::BLUE_ROBOT && extract_robot(object.robot_id, false, robot)) || 
           (object.type == ConstraintObject::YELLOW_ROBOT && extract_robot(object.robot_id, true, robot))){
    object_pose.x = robot.pos.x;
    object_pose.y = robot.pos.y;
    object_pose.theta = robot.orientation;
    return true;
  }

  return false;
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
