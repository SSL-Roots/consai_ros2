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
#include <memory>
#include <vector>

#include "consai_robot_controller/constraint_parser.hpp"
#include "consai_tools/geometry_tools.hpp"
#include "robocup_ssl_msgs/msg/robot_id.hpp"
#include "robocup_ssl_msgs/msg/tracked_ball.hpp"
#include "robocup_ssl_msgs/msg/tracked_frame.hpp"
#include "robocup_ssl_msgs/msg/tracked_robot.hpp"

namespace parser
{

namespace tools = geometry_tools;

using TrackedBall = robocup_ssl_msgs::msg::TrackedBall;
using TrackedFrame = robocup_ssl_msgs::msg::TrackedFrame;
using TrackedRobot = robocup_ssl_msgs::msg::TrackedRobot;

ConstraintParser::ConstraintParser(
  const std::shared_ptr<DetectionExtractor> & detection_extractor,
  const bool team_is_yellow)
{
  detection_ = detection_extractor;
  team_is_yellow_ = team_is_yellow;
}

void ConstraintParser::set_field_size(const double field_length, const double field_width)
{
  field_half_length_ = field_length / 2.0;
  field_half_width_ = field_width / 2.0;
}

void ConstraintParser::set_named_targets(const NamedTargets::SharedPtr msg)
{
  // トピックを受け取るたびに初期化する
  named_targets_.clear();

  for (std::size_t i = 0; i < msg->name.size(); ++i) {
    auto name = msg->name[i];
    auto pose = msg->pose[i];
    named_targets_[name] = pose;
  }
}

bool ConstraintParser::parse_constraint_object(
  const ConstraintObject & object,
  State & object_pose) const
{
  TrackedBall ball;
  TrackedRobot robot;

  // NOLINTについて
  // ament_uncrustifyとament_cpplintが競合するので、lintのチェックをスキップする
  // Ref: https://github.com/ament/ament_lint/issues/158
  if (object.type == ConstraintObject::BALL && detection_->extract_ball(ball)) {
    object_pose.x = ball.pos.x;
    object_pose.y = ball.pos.y;
    return true;
  } else if (  // NOLINT
    (object.type == ConstraintObject::BLUE_ROBOT &&
    detection_->extract_robot(object.robot_id, false, robot)) ||
    (object.type == ConstraintObject::YELLOW_ROBOT &&
    detection_->extract_robot(object.robot_id, true, robot)) ||
    (object.type == ConstraintObject::OUR_ROBOT &&
    detection_->extract_robot(object.robot_id, team_is_yellow_, robot)) ||
    (object.type == ConstraintObject::THEIR_ROBOT &&
    detection_->extract_robot(object.robot_id, !team_is_yellow_, robot)))
  {
    object_pose.x = robot.pos.x;
    object_pose.y = robot.pos.y;
    object_pose.theta = robot.orientation;
    return true;
  } else if (object.type == ConstraintObject::NAMED_TARGET &&  // NOLINT
    named_targets_.find(object.name) != named_targets_.end())
  {
    object_pose = named_targets_.at(object.name);
    return true;
  } else if (object.type == ConstraintObject::OUR_GOAL) {
    object_pose.x = -field_half_length_;
    object_pose.y = 0.0;
    return true;
  } else if (object.type == ConstraintObject::THEIR_GOAL) {
    object_pose.x = field_half_length_;
    object_pose.y = 0.0;
    return true;
  }

  return false;
}

bool ConstraintParser::parse_constraint_xy(
  const ConstraintXY & xy, double & parsed_x,
  double & parsed_y) const
{
  State object_pose;
  if (xy.object.size() > 0) {
    if (parse_constraint_object(xy.object[0], object_pose)) {
      parsed_x = object_pose.x;
      parsed_y = object_pose.y;
    } else {
      return false;
    }
  }

  if (xy.value_x.size() > 0) {
    parsed_x = xy.value_x[0];
  }

  if (xy.value_y.size() > 0) {
    parsed_y = xy.value_y[0];
  }

  // フィールドサイズに対してx, yが-1 ~ 1に正規化されている
  if (xy.normalized) {
    parsed_x *= field_half_length_;
    parsed_y *= field_half_width_;
  }
  return true;
}

bool ConstraintParser::parse_constraint_theta(
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

bool ConstraintParser::parse_constraint_pose(const ConstraintPose & pose, State & parsed_pose) const
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

bool ConstraintParser::parse_constraint_line(
  const ConstraintLine & line, State & parsed_pose) const
{
  State p1, p2;
  if (!parse_constraint_xy(line.p1, p1.x, p1.y)) {
    return false;
  }
  if (!parse_constraint_xy(line.p2, p2.x, p2.y)) {
    return false;
  }

  State p3, p4;
  bool has_p3_p4 = false;
  if (line.p3.size() > 0 && line.p4.size() > 0) {
    if (parse_constraint_xy(line.p3[0], p3.x, p3.y) &&
      parse_constraint_xy(line.p4[0], p4.x, p4.y))
    {
      has_p3_p4 = true;
    }
  }

  // 直線p1->p2の座標系を作成
  auto angle_p1_to_p2 = tools::calc_angle(p1, p2);
  tools::Trans trans_1to2(p1, angle_p1_to_p2);
  if (has_p3_p4) {
    // p1 ~ p4がセットされていれば、
    // 直線p1->p2上で、直線p3->p4と交わるところを目標位置とする
    State intersection = tools::intersection(p1, p2, p3, p4);
    // 交点が取れなければp1を目標位置とする
    if (!std::isfinite(intersection.x) || !std::isfinite(intersection.y)) {
      parsed_pose = p1;
    } else {
      auto parsed_pose_1to2 = trans_1to2.transform(intersection);
      auto p2_1to2 = trans_1to2.transform(p2);
      // 交点が直線p1->p2をはみ出る場合は、p1 or p2に置き換える
      if (parsed_pose_1to2.x < 0.0) {
        parsed_pose_1to2.x = 0.0;
      } else if (parsed_pose_1to2.x > p2_1to2.x) {
        parsed_pose_1to2.x = p2_1to2.x;
      }
      // オフセットを加算する。オフセットによりp1, p2をはみ出ることが可能
      if (line.offset_intersection_to_p2.size() > 0) {
        parsed_pose_1to2.x += line.offset_intersection_to_p2[0];
      }
      // 座標系をもとに戻す
      parsed_pose = trans_1to2.inverted_transform(parsed_pose_1to2);
    }
  } else {
    // 直線p1->p2上で、p1からdistanceだけ離れた位置を目標位置する
    parsed_pose = trans_1to2.inverted_transform(line.distance, 0, 0);
  }

  if (!parse_constraint_theta(
      line.theta, parsed_pose.x, parsed_pose.y,
      parsed_pose.theta))
  {
    return false;
  }

  return true;
}


}  // namespace parser
