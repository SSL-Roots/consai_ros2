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
#include <string>
#include <vector>

#include "control_toolbox/pid.hpp"
#include "consai_msgs/action/robot_control.hpp"
#include "consai_msgs/msg/robot_command.hpp"
#include "consai_msgs/msg/state2_d.hpp"
#include "consai_msgs/srv/stop_control.hpp"
#include "consai_robot_controller/field_info_parser.hpp"
#include "consai_robot_controller/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robocup_ssl_msgs/msg/geometry_data.hpp"
#include "robocup_ssl_msgs/msg/tracked_frame.hpp"
#include "robocup_ssl_msgs/msg/tracked_robot.hpp"

namespace consai_robot_controller
{
using State = consai_msgs::msg::State2D;
using RobotControl = consai_msgs::action::RobotControl;
using GoalHandleRobotControl = rclcpp_action::ServerGoalHandle<RobotControl>;
using GeometryData = robocup_ssl_msgs::msg::GeometryData;
using TrackedFrame = robocup_ssl_msgs::msg::TrackedFrame;
using TrackedRobot = robocup_ssl_msgs::msg::TrackedRobot;
using StopControl = consai_msgs::srv::StopControl;


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
  bool update_pid_gain_from_param(
    const rclcpp::Parameter & param,
    const std::string & prefix,
    std::vector<std::shared_ptr<control_toolbox::Pid>> & pid_controller);
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const RobotControl::Goal> goal,
    const unsigned int robot_id);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleRobotControl> goal_handle,
    const unsigned int robot_id);
  void handle_accepted(
    std::shared_ptr<GoalHandleRobotControl> goal_handle,
    const unsigned int robot_id);
  void handle_stop_control(
    const std::shared_ptr<StopControl::Request> request,
    std::shared_ptr<StopControl::Response> response);
  State limit_world_velocity(const State & velocity) const;
  State limit_world_acceleration(
    const State & velocity, const State & last_velocity,
    const rclcpp::Duration & dt) const;
  bool arrived(const TrackedRobot & my_robot, const State & goal_pose);
  bool switch_to_stop_control_mode(
    const unsigned int robot_id, const bool success, const std::string & error_msg);

  std::vector<rclcpp::Publisher<consai_msgs::msg::RobotCommand>::SharedPtr> pub_command_;
  std::vector<rclcpp::Publisher<State>::SharedPtr> pub_goal_pose_;
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

  consai_robot_controller::FieldInfoParser parser_;
  rclcpp::Subscription<TrackedFrame>::SharedPtr sub_detection_tracked_;
  rclcpp::Subscription<GeometryData>::SharedPtr sub_geometry_;
  rclcpp::Service<StopControl>::SharedPtr server_stop_control_;
  bool team_is_yellow_;
  rclcpp::Clock steady_clock_;
  OnSetParametersCallbackHandle::SharedPtr handler_change_param_;
  double max_acceleration_xy_;
  double max_acceleration_theta_;
  double max_velocity_xy_;
  double max_velocity_theta_;
};

}  // namespace consai_robot_controller

#endif  // CONSAI_ROBOT_CONTROLLER__CONTROLLER_COMPONENT_HPP_
