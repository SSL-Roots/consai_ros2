// Copyright 2025 Roots
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

#ifndef CONSAI_ROBOT_CONTROLLER__CONTROLLER_UNIT_HPP_
#define CONSAI_ROBOT_CONTROLLER__CONTROLLER_UNIT_HPP_

#include <memory>
#include <optional>
#include <string>

#include "consai_frootspi_msgs/msg/robot_command.hpp"
#include "consai_msgs/msg/state2_d.hpp"
#include "consai_robot_controller/control_params.hpp"
#include "consai_robot_controller/locomotion_controller.hpp"
#include "consai_robot_controller/trajectory/utils.hpp"
#include "robocup_ssl_msgs/msg/tracked_robot.hpp"
#include "rclcpp/rclcpp.hpp"

namespace consai_robot_controller
{

using RobotCommand = consai_frootspi_msgs::msg::RobotCommand;
using TrackedRobot = robocup_ssl_msgs::msg::TrackedRobot;
using State = consai_msgs::msg::State2D;

class ControllerUnit
{
 public:
  ControllerUnit(unsigned int robot_id, bool team_is_yellow, double dt)
  : robot_id_(robot_id), team_is_yellow_(team_is_yellow),
    locomotion_controller_(LocomotionController(robot_id, dt)) {}
  void set_robot_command_publisher(rclcpp::Node::SharedPtr node, const std::string & topic_name);
  void set_debug_publishers(rclcpp::Node::SharedPtr node);
  void move_to_desired_pose(
    const Pose2D & goal_pose,
    const TrackedRobot & my_robot,
    const double kick_power, const double dribble_power,
    std::optional<double> limit_vel_xy = std::nullopt);
  void publish_stop_command(void);
  void set_control_params(const ControlParams & control_params);
  void publish_debug_data(const TrackedRobot & my_robot);

 private:
  double calculate_angular_velocity(
    const Velocity2D & desired_velocity, const Pose2D & goal_pose, const TrackedRobot & my_robot);

  rclcpp::Publisher<RobotCommand>::SharedPtr pub_command_;
  unsigned int robot_id_;
  bool team_is_yellow_;
  LocomotionController locomotion_controller_;
  ControlParams desired_control_params_;
  Velocity2D world_vel_;

  rclcpp::Publisher<State>::SharedPtr pub_debug_current_pose_;
  rclcpp::Publisher<State>::SharedPtr pub_debug_current_vel_;
  rclcpp::Publisher<State>::SharedPtr pub_debug_goal_pose_;
  rclcpp::Publisher<State>::SharedPtr pub_debug_target_speed_world_;
  rclcpp::Publisher<State>::SharedPtr pub_debug_control_output_ff_;
  rclcpp::Publisher<State>::SharedPtr pub_debug_control_output_p_;
};

}  // namespace consai_robot_controller

#endif  // CONSAI_ROBOT_CONTROLLER__CONTROLLER_UNIT_HPP_