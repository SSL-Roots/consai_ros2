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

#ifndef CONSAI_ROBOT_CONTROLLER__GRSIM_COMMAND_CONVERTER_HPP_
#define CONSAI_ROBOT_CONTROLLER__GRSIM_COMMAND_CONVERTER_HPP_

#include <memory>
#include <vector>

#include "consai_frootspi_msgs/msg/robot_command.hpp"
#include "consai_robot_controller/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "robocup_ssl_msgs/msg/commands.hpp"
#include "robocup_ssl_msgs/msg/robot_command.hpp"

namespace consai_robot_controller
{

using ConsaiCommand = consai_frootspi_msgs::msg::RobotCommand;
using GrSimCommands = robocup_ssl_msgs::msg::Commands;
using GrSimRobotCommand = robocup_ssl_msgs::msg::RobotCommand;

class GrSimCommandConverter : public rclcpp::Node
{
public:
  CONSAI_ROBOT_CONTROLLER_PUBLIC
  explicit GrSimCommandConverter(const rclcpp::NodeOptions & options);

protected:
  void on_timer();

private:
  void callback_consai_command_(const ConsaiCommand::SharedPtr msg);

  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<rclcpp::Subscription<ConsaiCommand>::SharedPtr> subs_consai_command_;
  rclcpp::Publisher<GrSimCommands>::SharedPtr pub_grsim_commands_;
  std::vector<ConsaiCommand::SharedPtr> consai_commands_;
};

}  // namespace consai_robot_controller

#endif  // CONSAI_ROBOT_CONTROLLER__GRSIM_COMMAND_CONVERTER_HPP_
