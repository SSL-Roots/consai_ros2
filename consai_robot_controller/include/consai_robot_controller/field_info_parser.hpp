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

#ifndef CONSAI_ROBOT_CONTROLLER__FIELD_INFO_PARSER_HPP_
#define CONSAI_ROBOT_CONTROLLER__FIELD_INFO_PARSER_HPP_

#include <memory>
#include <vector>

#include "consai_msgs/msg/constraint_object.hpp"
#include "consai_msgs/msg/constraint_pose.hpp"
#include "consai_msgs/msg/constraint_theta.hpp"
#include "consai_msgs/msg/constraint_xy.hpp"
#include "consai_msgs/action/robot_control.hpp"
#include "consai_msgs/msg/state2_d.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robocup_ssl_msgs/msg/geometry_data.hpp"
#include "robocup_ssl_msgs/msg/tracked_ball.hpp"
#include "robocup_ssl_msgs/msg/tracked_frame.hpp"
#include "robocup_ssl_msgs/msg/tracked_robot.hpp"

namespace consai_robot_controller
{

using namespace robocup_ssl_msgs::msg;
using namespace consai_msgs::msg;
using RobotControl = consai_msgs::action::RobotControl;
using State = consai_msgs::msg::State2D;

class FieldInfoParser
{
public:
  FieldInfoParser();
  void set_detection_tracked(const TrackedFrame::SharedPtr detection_tracked);
  void set_geometry(const GeometryData::SharedPtr geometry);
  bool extract_robot(const unsigned int robot_id, const bool team_is_yellow, TrackedRobot & my_robot) const;
  bool extract_ball(TrackedBall & my_ball) const;
  bool parse_goal(const std::shared_ptr<const RobotControl::Goal> goal, State & parsed_pose) const;
  bool parse_goal(const std::shared_ptr<const RobotControl::Goal> goal,
                  const TrackedRobot & my_robot, State & parsed_pose,
                  double & kick_power, double & dribble_power) const;
  bool parse_constraint_pose(const ConstraintPose & pose, State & parsed_pose) const;
  bool parse_constraint_xy(const ConstraintXY & xy, double & parsed_x, double & parsed_y) const;
  bool parse_constraint_theta(const ConstraintTheta & theta, const double goal_x, const double goal_y, double & parsed_theta) const;
  bool parse_constraint_object(const ConstraintObject & object, State & object_pose) const;
  double calc_angle(const State & from_pose, const State & to_pose) const;
  double normalize_theta(const double theta) const;

private:
  std::shared_ptr<TrackedFrame> detection_tracked_;
  std::shared_ptr<GeometryData> geometry_;
};

}  // namespace consai_robot_controller

#endif  // CONSAI_ROBOT_CONTROLLER__FIELD_INFO_PARSER_HPP_
