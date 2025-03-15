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

#include "consai_robot_controller/controller_unit.hpp"

namespace consai_robot_controller
{

void ControllerUnit::set_robot_command_publisher(
  rclcpp::Node::SharedPtr node, const std::string & topic_name)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
    // .best_effort()  // データの損失を許容する
    .durability_volatile();  // データの保存を行わない

  pub_command_ = node->create_publisher<RobotCommand>(topic_name, qos);
}


void ControllerUnit::publish_robot_command(RobotCommand::UniquePtr msg)
{
  pub_command_->publish(std::move(msg));
}


}  // namespace consai_robot_controller