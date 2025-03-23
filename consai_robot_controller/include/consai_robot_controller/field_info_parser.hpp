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

#include "consai_msgs/msg/named_targets.hpp"
#include "consai_msgs/msg/robot_control_msg.hpp"
#include "consai_msgs/msg/state2_d.hpp"
#include "consai_robot_controller/constraint_parser.hpp"
#include "consai_robot_controller/detection_extractor.hpp"
#include "consai_robot_controller/obstacle/obstacle_environment.hpp"
#include "consai_robot_controller/tactic/back_dribble_tactics.hpp"
#include "consai_robot_controller/tactic/ball_boy_tactics.hpp"
#include "consai_robot_controller/tactic/dribble_tactics.hpp"
#include "consai_robot_controller/tactic/shoot_tactics.hpp"
#include "consai_robot_controller/tactic/tactic_control_ball.hpp"
#include "consai_robot_controller/tactic/tactic_obstacle_avoidance.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robocup_ssl_msgs/msg/tracked_ball.hpp"
#include "robocup_ssl_msgs/msg/tracked_frame.hpp"
#include "robocup_ssl_msgs/msg/tracked_robot.hpp"

namespace consai_robot_controller
{

using RobotControlMsg = consai_msgs::msg::RobotControlMsg;
using NamedTargets = consai_msgs::msg::NamedTargets;
using State = consai_msgs::msg::State2D;
using TrackedBall = robocup_ssl_msgs::msg::TrackedBall;
using TrackedFrame = robocup_ssl_msgs::msg::TrackedFrame;
using TrackedRobot = robocup_ssl_msgs::msg::TrackedRobot;

class FieldInfoParser
{
public:
  FieldInfoParser(
    const bool team_is_yellow, const bool invert,
    const std::shared_ptr<parser::DetectionExtractor> & detection_extractor);
  void set_subscriptions(rclcpp::Node* node);
  void set_consai_param_rule(const nlohmann::json & param);
  void set_detection_tracked(const TrackedFrame::SharedPtr detection_tracked);
  void set_named_targets(const NamedTargets::SharedPtr msg);
  bool is_parsable(const RobotControlMsg::SharedPtr goal) const;
  bool parse_goal(
    const RobotControlMsg::SharedPtr goal,
    const TrackedRobot & my_robot, State & parsed_pose, State & final_goal_pose,
    double & kick_power, double & dribble_power);
  State modify_goal_pose_to_avoid_obstacles(
    const RobotControlMsg::SharedPtr goal,
    const TrackedRobot & my_robot,
    const State & goal_pose, const State & final_goal_pose) const;

private:
  bool parse_constraints(
    const RobotControlMsg::SharedPtr goal, State & parsed_pose) const;

  rclcpp::Subscription<TrackedFrame>::SharedPtr sub_detection_tracked_;
  rclcpp::Subscription<NamedTargets>::SharedPtr sub_named_targets_;

  bool team_is_yellow_ = false;
  bool invert_ = false;
  std::shared_ptr<parser::DetectionExtractor> detection_extractor_;

  tactics::BallBoyTactics ball_boy_tactics_;
  dribble_tactics::DribbleTactics dribble_tactics_;
  back_dribble_tactics::BackDribbleTactics back_dribble_tactics_;
  shoot_tactics::ShootTactics shoot_tactics_;
  std::shared_ptr<parser::ConstraintParser> constraint_parser_;
  std::shared_ptr<tactic::ControlBall> tactic_control_ball_;
  std::shared_ptr<tactic::ObstacleAvoidance> tactic_obstacle_avoidance_;
};

}  // namespace consai_robot_controller

#endif  // CONSAI_ROBOT_CONTROLLER__FIELD_INFO_PARSER_HPP_
