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

#include "consai_frootspi_msgs/msg/robot_command.hpp"
#include "rclcpp/rclcpp.hpp"

namespace consai_robot_controller
{

using RobotCommand = consai_frootspi_msgs::msg::RobotCommand;

class ControllerUnit
{

public:
  void set_robot_command_publisher(rclcpp::Node::SharedPtr node, const std::string & topic_name);
  void publish_robot_command(RobotCommand::UniquePtr msg);


private:

  rclcpp::Publisher<RobotCommand>::SharedPtr pub_command_;


};

}  // namespace consai_robot_controller

#endif  // CONSAI_ROBOT_CONTROLLER__CONTROLLER_UNIT_HPP_