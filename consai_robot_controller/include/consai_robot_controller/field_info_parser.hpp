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
#include <string>

#include "consai_msgs/action/robot_control.hpp"
#include "consai_msgs/msg/named_targets.hpp"
#include "consai_msgs/msg/parsed_referee.hpp"
#include "consai_msgs/msg/state2_d.hpp"
#include "consai_robot_controller/ball_boy_tactics.hpp"
#include "consai_robot_controller/constraint_parser.hpp"
#include "consai_robot_controller/detection_extractor.hpp"
#include "consai_robot_controller/obstacle_environment.hpp"
#include "consai_robot_controller/tactic_control_ball.hpp"
#include "consai_robot_controller/tactic_obstacle_avoidance.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robocup_ssl_msgs/msg/geometry_data.hpp"
#include "robocup_ssl_msgs/msg/referee.hpp"
#include "robocup_ssl_msgs/msg/tracked_ball.hpp"
#include "robocup_ssl_msgs/msg/tracked_frame.hpp"
#include "robocup_ssl_msgs/msg/tracked_robot.hpp"

namespace consai_robot_controller
{

using RobotControl = consai_msgs::action::RobotControl;
using NamedTargets = consai_msgs::msg::NamedTargets;
using State = consai_msgs::msg::State2D;
using GeometryData = robocup_ssl_msgs::msg::GeometryData;
using ParsedReferee = consai_msgs::msg::ParsedReferee;
using Referee = robocup_ssl_msgs::msg::Referee;
using TrackedBall = robocup_ssl_msgs::msg::TrackedBall;
using TrackedFrame = robocup_ssl_msgs::msg::TrackedFrame;
using TrackedRobot = robocup_ssl_msgs::msg::TrackedRobot;

class FieldInfoParser
{
public:
  FieldInfoParser(
    const bool team_is_yellow, const bool invert,
    const std::shared_ptr<parser::DetectionExtractor> & detection_extractor);
  void set_detection_tracked(const TrackedFrame::SharedPtr detection_tracked);
  void set_geometry(const GeometryData::SharedPtr geometry);
  void set_referee(const Referee::SharedPtr referee);
  void set_parsed_referee(const ParsedReferee::SharedPtr parsed_referee);
  void set_named_targets(const NamedTargets::SharedPtr msg);
  bool parse_goal(const std::shared_ptr<const RobotControl::Goal> goal, State & parsed_pose) const;
  bool parse_goal(
    const std::shared_ptr<const RobotControl::Goal> goal,
    const TrackedRobot & my_robot, State & parsed_pose, State & final_goal_pose,
    double & kick_power, double & dribble_power);
  State modify_goal_pose_to_avoid_obstacles(
    const std::shared_ptr<const RobotControl::Goal> goal,
    const TrackedRobot & my_robot,
    const State & goal_pose, const State & final_goal_pose) const;

private:
  bool parse_kick(
    const State & kick_target, const TrackedRobot & my_robot, const TrackedBall & ball,
    const bool & kick_pass, const bool & kick_setplay,
    State & parsed_pose, double & parsed_kick_power, double & parsed_dribble_power) const;
  bool parse_dribble(
    const State & dribble_target, const TrackedRobot & my_robot, const TrackedBall & ball,
    State & parsed_pose, double & parsed_dribble_power) const;

  bool team_is_yellow_ = false;
  bool invert_ = false;
  std::shared_ptr<parser::DetectionExtractor> detection_extractor_;

  std::shared_ptr<Referee> referee_;
  std::shared_ptr<ParsedReferee> parsed_referee_;
  tactics::BallBoyTactics ball_boy_tactics_;
  std::shared_ptr<parser::ConstraintParser> constraint_parser_;
  std::shared_ptr<tactic::ControlBall> tactic_control_ball_;
  std::shared_ptr<tactic::ObstacleAvoidance> tactic_obstacle_avoidance_;
};

}  // namespace consai_robot_controller

#endif  // CONSAI_ROBOT_CONTROLLER__FIELD_INFO_PARSER_HPP_
