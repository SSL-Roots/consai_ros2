// Copyright 2024 Roots
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

#ifndef CONSAI_ROBOT_CONTROLLER__VSSL_COMMAND_SENDER_HPP_
#define CONSAI_ROBOT_CONTROLLER__VSSL_COMMAND_SENDER_HPP_

#include <memory>
#include <vector>

#include "consai_frootspi_msgs/msg/robot_command.hpp"
#include "consai_robot_controller/visibility_control.h"

#include "robocup_ssl_comm/udp_sender.hpp"
#include "robocup_ssl_msgs/vssl_robot_control.pb.h"

#include "rclcpp/rclcpp.hpp"

namespace consai_robot_controller
{

using ConsaiCommand = consai_frootspi_msgs::msg::RobotCommand;

class VsslCommandSender: public rclcpp::Node
{
public:
  CONSAI_ROBOT_CONTROLLER_PUBLIC
  explicit VsslCommandSender(const rclcpp::NodeOptions & options);

private:
  void callback_consai_command(const ConsaiCommand::SharedPtr msg, const int id);

  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::unique_ptr<udp_sender::UDPSender>> senders_;
  std::vector<rclcpp::Subscription<ConsaiCommand>::SharedPtr> subs_consai_command_;
};

}  // namespace consai_robot_controller

#endif  // CONSAI_ROBOT_CONTROLLER__VSSL_COMMAND_SENDER_HPP_
