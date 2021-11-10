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

#ifndef ROBOCUP_SSL_COMM__GRSIM_COMPONENT_HPP_
#define ROBOCUP_SSL_COMM__GRSIM_COMPONENT_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "robocup_ssl_comm/udp_sender.hpp"
#include "robocup_ssl_comm/visibility_control.h"
#include "robocup_ssl_msgs/grSim_Commands.pb.h"
#include "robocup_ssl_msgs/grSim_Replacement.pb.h"
#include "robocup_ssl_msgs/msg/commands.hpp"
#include "robocup_ssl_msgs/msg/replacement.hpp"
#include "robocup_ssl_msgs/msg/robot_command.hpp"
#include "robocup_ssl_msgs/msg/robot_replacement.hpp"

namespace robocup_ssl_comm
{

using Commands = robocup_ssl_msgs::msg::Commands;
using RobotCommand = robocup_ssl_msgs::msg::RobotCommand;
using Replacement = robocup_ssl_msgs::msg::Replacement;
using RobotReplacement = robocup_ssl_msgs::msg::RobotReplacement;

class GrSim : public rclcpp::Node
{
public:
  ROBOCUP_SSL_COMM_PUBLIC
  explicit GrSim(const rclcpp::NodeOptions & options);

protected:
  void on_timer();

private:
  void callback_commands(const Commands::SharedPtr msg);
  void callback_single_command(const RobotCommand::SharedPtr msg);
  void callback_replacement(const Replacement::SharedPtr msg);

  void set_command(grSim_Robot_Command * robot_command, const RobotCommand & msg_robot_command);
  void set_robot_replacement(
    grSim_RobotReplacement * robot_replacement,
    const RobotReplacement & msg_robot_replacement);

  std::unique_ptr<udp_sender::UDPSender> sender_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<Commands>::SharedPtr sub_commands_;
  rclcpp::Subscription<Replacement>::SharedPtr sub_replacement_;
};

}  // namespace robocup_ssl_comm

#endif  // ROBOCUP_SSL_COMM__GRSIM_COMPONENT_HPP_
