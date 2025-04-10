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

#ifndef CONSAI_ROBOT_CONTROLLER__CONSTRAINT_PARSER_HPP_
#define CONSAI_ROBOT_CONTROLLER__CONSTRAINT_PARSER_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "consai_msgs/msg/constraint_line.hpp"
#include "consai_msgs/msg/constraint_object.hpp"
#include "consai_msgs/msg/constraint_pose.hpp"
#include "consai_msgs/msg/constraint_theta.hpp"
#include "consai_msgs/msg/constraint_xy.hpp"
#include "consai_msgs/msg/named_targets.hpp"
#include "consai_msgs/msg/state2_d.hpp"
#include "consai_robot_controller/detection_extractor.hpp"
#include "rclcpp/rclcpp.hpp"


namespace parser
{

using ConstraintLine = consai_msgs::msg::ConstraintLine;
using ConstraintObject = consai_msgs::msg::ConstraintObject;
using ConstraintPose = consai_msgs::msg::ConstraintPose;
using ConstraintTheta = consai_msgs::msg::ConstraintTheta;
using ConstraintXY = consai_msgs::msg::ConstraintXY;
using NamedTargets = consai_msgs::msg::NamedTargets;
using State = consai_msgs::msg::State2D;

using DetectionExtractor = parser::DetectionExtractor;

class ConstraintParser
{
public:
  explicit ConstraintParser(
    const std::shared_ptr<DetectionExtractor> & detection_extractor,
    const bool team_is_yellow);

  void set_field_size(const double field_length, const double field_width);
  void set_named_targets(const NamedTargets::SharedPtr msg);
  bool parse_constraint_object(const ConstraintObject & object, State & object_pose) const;
  bool parse_constraint_xy(const ConstraintXY & xy, double & parsed_x, double & parsed_y) const;
  bool parse_constraint_theta(
    const ConstraintTheta & theta, const double goal_x,
    const double goal_y, double & parsed_theta) const;
  bool parse_constraint_pose(const ConstraintPose & pose, State & parsed_pose) const;
  bool parse_constraint_line(const ConstraintLine & line, State & parsed_pose) const;

private:
  std::shared_ptr<DetectionExtractor> detection_;
  std::map<std::string, State> named_targets_;

  bool team_is_yellow_;
  double field_half_length_ = 6.0;
  double field_half_width_ = 4.5;
};

}  // namespace parser


#endif  // CONSAI_ROBOT_CONTROLLER__CONSTRAINT_PARSER_HPP_
