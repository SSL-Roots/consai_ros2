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

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "consai_msgs/action/robot_control.hpp"
#include "consai_msgs/msg/constraint_line.hpp"
#include "consai_msgs/msg/constraint_object.hpp"
#include "consai_msgs/msg/constraint_pose.hpp"
#include "consai_msgs/msg/constraint_theta.hpp"
#include "consai_msgs/msg/constraint_xy.hpp"
#include "consai_msgs/msg/named_targets.hpp"
#include "consai_msgs/msg/parsed_referee.hpp"
#include "consai_msgs/msg/state2_d.hpp"
#include "consai_robot_controller/obstacle_environment.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robocup_ssl_msgs/msg/geometry_data.hpp"
#include "robocup_ssl_msgs/msg/referee.hpp"
#include "robocup_ssl_msgs/msg/tracked_ball.hpp"
#include "robocup_ssl_msgs/msg/tracked_frame.hpp"
#include "robocup_ssl_msgs/msg/tracked_robot.hpp"

namespace consai_robot_controller
{

using RobotControl = consai_msgs::action::RobotControl;
using ConstraintLine = consai_msgs::msg::ConstraintLine;
using ConstraintObject = consai_msgs::msg::ConstraintObject;
using ConstraintPose = consai_msgs::msg::ConstraintPose;
using ConstraintTheta = consai_msgs::msg::ConstraintTheta;
using ConstraintXY = consai_msgs::msg::ConstraintXY;
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
  FieldInfoParser();
  void set_invert(const bool & invert);
  void set_team_is_yellow(const bool & team_is_yellow);
  void set_detection_tracked(const TrackedFrame::SharedPtr detection_tracked);
  void set_geometry(const GeometryData::SharedPtr geometry);
  void set_referee(const Referee::SharedPtr referee);
  void set_parsed_referee(const ParsedReferee::SharedPtr parsed_referee);
  void set_named_targets(const NamedTargets::SharedPtr msg);
  bool extract_robot(
    const unsigned int robot_id, const bool team_is_yellow,
    TrackedRobot & my_robot) const;
  bool extract_ball(TrackedBall & my_ball) const;
  bool parse_goal(const std::shared_ptr<const RobotControl::Goal> goal, State & parsed_pose) const;
  bool parse_goal(
    const std::shared_ptr<const RobotControl::Goal> goal,
    const TrackedRobot & my_robot, State & parsed_pose, State & final_goal_pose,
    double & kick_power, double & dribble_power) const;
  std::vector<unsigned int> active_robot_id_list(const bool team_is_yellow) const;
  State modify_goal_pose_to_avoid_obstacles(
    const std::shared_ptr<const RobotControl::Goal> goal,
    const TrackedRobot & my_robot,
    const State & goal_pose, const State & final_goal_pose) const;
  obstacle::ObstacleEnvironment get_obstacle_environment(
    const std::shared_ptr<const RobotControl::Goal> goal,
    const TrackedRobot & my_robot) const;

  // controller_componentからアクセスするためpublic変数で定義
  double param_threshold_looking_ball_distance;
  double param_threshold_looking_ball_theta;
  double param_can_dribble_distance;
  double param_can_shoot_theta;
  double param_distance_to_look_ball;
  double param_distance_to_rotate;

private:
  bool parse_constraint_pose(const ConstraintPose & pose, State & parsed_pose) const;
  bool parse_constraint_line(const ConstraintLine & line, State & parsed_pose) const;
  bool parse_constraint_xy(const ConstraintXY & xy, double & parsed_x, double & parsed_y) const;
  bool parse_constraint_theta(
    const ConstraintTheta & theta, const double goal_x,
    const double goal_y, double & parsed_theta) const;
  bool parse_constraint_object(const ConstraintObject & object, State & object_pose) const;
  bool parse_kick(
    const State & kick_target, const TrackedRobot & my_robot, const TrackedBall & ball,
    const bool & kick_pass, const bool & kick_setplay,
    State & parsed_pose, double & parsed_kick_power, double & parsed_dribble_power) const;
  bool parse_dribble(
    const State & dribble_target, const TrackedRobot & my_robot, const TrackedBall & ball,
    State & parsed_pose, double & parsed_dribble_power) const;
  bool control_ball(
    const State & target, const TrackedRobot & my_robot, const TrackedBall & ball,
    const double & dribble_distance, State & parsed_pose, bool & need_kick,
    bool & need_dribble) const;
  bool control_ball_at_setplay(
    const State & target, const TrackedRobot & my_robot, const TrackedBall & ball,
    State & parsed_pose, bool & need_kick, bool & need_dribble) const;
  bool receive_ball(
    const TrackedRobot & my_robot, const TrackedBall & ball,
    State & parsed_pose, double & parsed_dribble_power) const;
  bool reflect_kick(
    const State & target, const TrackedRobot & my_robot, const TrackedBall & ball,
    const bool & kick_pass, State & parsed_pose, double & parsed_kick_power,
    double & parsed_dribble_power) const;
  bool avoid_obstacles(
    const TrackedRobot & my_robot, const State & goal_pose, const TrackedBall & ball,
    const bool & avoid_ball, State & avoidance_pose) const;
  bool avoid_placement_area(
    const TrackedRobot & my_robot, const State & goal_pose, const TrackedBall & ball,
    const bool avoid_kick_receive_area,
    const State & designated_position, State & avoidance_pose) const;
  bool avoid_robots(
    const TrackedRobot & my_robot, const State & goal_pose,
    State & avoidance_pose) const;
  bool avoid_ball_500mm(
    const State & final_goal_pose,
    const State & goal_pose, const TrackedBall & ball,
    State & avoidance_pose) const;

  std::shared_ptr<TrackedFrame> detection_tracked_;
  std::shared_ptr<GeometryData> geometry_;
  std::shared_ptr<Referee> referee_;
  std::shared_ptr<ParsedReferee> parsed_referee_;
  bool invert_;
  bool team_is_yellow_;
  std::map<std::string, State> named_targets_;
};

}  // namespace consai_robot_controller

#endif  // CONSAI_ROBOT_CONTROLLER__FIELD_INFO_PARSER_HPP_
