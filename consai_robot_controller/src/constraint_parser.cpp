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
#include "consai_robot_controller/geometry_tools.hpp"
#include "consai_robot_controller/obstacle_ball.hpp"
#include "consai_robot_controller/obstacle_environment.hpp"
#include "consai_robot_controller/obstacle_robot.hpp"
#include "consai_robot_controller/obstacle_typedef.hpp"
#include "consai_robot_controller/prohibited_area.hpp"
#include "robocup_ssl_msgs/msg/robot_id.hpp"

namespace parser
{

ConstraintParser::ConstraintParser(
  const std::shared_ptr<DetectionExtractor> & detection_extractor,
  const bool team_is_yellow)
{
  detection_ = detection_extractor;
  team_is_yellow_ = team_is_yellow;
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

bool ConstraintParser::parse_constraint_object(const ConstraintObject & object, State & object_pose) const
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
    (object.type == ConstraintObject::BLUE_ROBOT && detection_->extract_robot(object.robot_id, false, robot)) ||
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


}  // namespace parser
