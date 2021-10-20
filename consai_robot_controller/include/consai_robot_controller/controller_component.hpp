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
#include "consai_msgs/msg/robot_command.hpp"
#include "consai_msgs/action/robot_control.hpp"
#include "consai_robot_controller/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robocup_ssl_msgs/msg/tracked_frame.hpp"
#include "robocup_ssl_msgs/msg/tracked_robot.hpp"

namespace consai_robot_controller
{
using RobotControl = consai_msgs::action::RobotControl;
using GoalHandleRobotControl = rclcpp_action::ServerGoalHandle<RobotControl>;
using namespace robocup_ssl_msgs::msg;


class Controller : public rclcpp::Node
{
public:
  CONSAI_ROBOT_CONTROLLER_PUBLIC
  explicit Controller(const rclcpp::NodeOptions & options);

protected:
  void on_timer();

private:
  void callback_detection_tracked(const TrackedFrame::SharedPtr msg);
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const RobotControl::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleRobotControl> goal_handle);
  void handle_accepted(std::shared_ptr<GoalHandleRobotControl> goal_handle);
  bool extract_my_robot(TrackedRobot & my_robot);
  bool arrived(const TrackedRobot & my_robot, const double x, const double y, const double theta);
  double normalize_theta(double theta);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<consai_msgs::msg::RobotCommand>::SharedPtr pub_command_; 
  rclcpp_action::Server<RobotControl>::SharedPtr server_control_;
  rclcpp::Subscription<TrackedFrame>::SharedPtr sub_detection_tracked_;
  std::shared_ptr<control_toolbox::Pid> pid_vx_;
  std::shared_ptr<control_toolbox::Pid> pid_vy_;
  std::shared_ptr<control_toolbox::Pid> pid_vtheta_;
  rclcpp::Clock steady_clock_;
  rclcpp::Time last_update_time_;
  unsigned int robot_id_;
  bool team_is_yellow_;
  std::shared_ptr<TrackedFrame> detection_tracked_;
  bool control_enable_;
  bool need_response_;
  std::shared_ptr<GoalHandleRobotControl> goal_handle_;
};

}  // namespace consai_robot_controller

#endif  // CONSAI_ROBOT_CONTROLLER__CONTROLLER_COMPONENT_HPP_
