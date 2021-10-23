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

#ifndef CONSAI_ROBOT_CONTROLLER__CONTROLLER_COMPONENT_HPP_
#define CONSAI_ROBOT_CONTROLLER__CONTROLLER_COMPONENT_HPP_

#include <memory>

#include "control_toolbox/pid.hpp"
#include "consai_msgs/msg/constraint_target.hpp"
#include "consai_msgs/msg/robot_command.hpp"
#include "consai_msgs/msg/state2_d.hpp"
#include "consai_msgs/action/robot_control.hpp"
#include "consai_robot_controller/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robocup_ssl_msgs/msg/geometry_data.hpp"
#include "robocup_ssl_msgs/msg/tracked_ball.hpp"
#include "robocup_ssl_msgs/msg/tracked_frame.hpp"
#include "robocup_ssl_msgs/msg/tracked_robot.hpp"

namespace consai_robot_controller
{
using ConstraintTarget = consai_msgs::msg::ConstraintTarget;
using State = consai_msgs::msg::State2D;
using RobotControl = consai_msgs::action::RobotControl;
using GoalHandleRobotControl = rclcpp_action::ServerGoalHandle<RobotControl>;
using namespace robocup_ssl_msgs::msg;


class Controller : public rclcpp::Node
{
public:
  CONSAI_ROBOT_CONTROLLER_PUBLIC
  explicit Controller(const rclcpp::NodeOptions & options);

protected:
  void on_timer_pub_control_command(const unsigned int robot_id);
  void on_timer_pub_stop_command(const unsigned int robot_id);

private:
  void callback_detection_tracked(const TrackedFrame::SharedPtr msg);
  void callback_geometry(const GeometryData::SharedPtr msg);
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const RobotControl::Goal> goal,
    const unsigned int robot_id);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleRobotControl> goal_handle,
    const unsigned int robot_id);
  void handle_accepted(std::shared_ptr<GoalHandleRobotControl> goal_handle, const unsigned int robot_id);
  bool parse_goal(const std::shared_ptr<const RobotControl::Goal> goal, State & parsed_pose);
  bool parse_constraint(const ConstraintTarget & target, const State & current_goal_pose, double & parsed_value) const;
  bool extract_robot(const unsigned int robot_id, const bool team_is_yellow, TrackedRobot & my_robot) const;
  bool extract_ball(TrackedBall & my_ball) const;
  State limit_world_velocity(const State & velocity) const;
  State limit_world_acceleration(const State & velocity, const State & last_velocity, const rclcpp::Duration & dt) const;
  bool arrived(const TrackedRobot & my_robot, const State & goal_pose);
  double calc_angle(const State & from_pose, const State & to_pose) const;
  double normalize_theta(double theta);

  std::vector<rclcpp::Publisher<consai_msgs::msg::RobotCommand>::SharedPtr> pub_command_; 
  std::vector<rclcpp_action::Server<RobotControl>::SharedPtr> server_control_;
  std::vector<rclcpp::Time> last_update_time_;
  std::vector<std::shared_ptr<control_toolbox::Pid>> pid_vx_;
  std::vector<std::shared_ptr<control_toolbox::Pid>> pid_vy_;
  std::vector<std::shared_ptr<control_toolbox::Pid>> pid_vtheta_;
  std::vector<rclcpp::TimerBase::SharedPtr> timer_pub_control_command_;
  std::vector<rclcpp::TimerBase::SharedPtr> timer_pub_stop_command_;
  std::vector<bool> control_enable_;
  std::vector<bool> need_response_;
  std::vector<std::shared_ptr<GoalHandleRobotControl>> goal_handle_;
  std::vector<State> last_world_vel_;

  rclcpp::Subscription<TrackedFrame>::SharedPtr sub_detection_tracked_;
  std::shared_ptr<TrackedFrame> detection_tracked_;
  rclcpp::Subscription<GeometryData>::SharedPtr sub_geometry_;
  std::shared_ptr<GeometryData> geometry_;
  bool team_is_yellow_;
  rclcpp::Clock steady_clock_;
};

}  // namespace consai_robot_controller

#endif  // CONSAI_ROBOT_CONTROLLER__CONTROLLER_COMPONENT_HPP_
