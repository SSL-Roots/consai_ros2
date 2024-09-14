// Copyright 2023 Roots
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

#include <memory>
#include <string>
#include <utility>

#include "consai_robot_controller/vssl_command_sender.hpp"

using namespace std::chrono_literals;

namespace consai_robot_controller
{

VsslCommandSender::VsslCommandSender(const rclcpp::NodeOptions & options)
: Node("vssl_command_sender", options)
{
  using namespace std::placeholders;

  declare_parameter("udp_address_base", "192.168.2.");
  declare_parameter("udp_port", 10000);

  for (int i = 0; i < 16; i++) {
    int addr_index = 20 + i;
    std::string addr = get_parameter("udp_address_base").as_string() + std::to_string(addr_index);
    senders_.push_back(
      std::make_unique<udp_sender::UDPSender>(
        addr,
        get_parameter("udp_port").as_int()));

    std::function<void(const ConsaiCommand::SharedPtr)> callback =
      std::bind(&VsslCommandSender::callback_consai_command, this, _1, i);
    auto sub_command = create_subscription<ConsaiCommand>(
      "robot" + std::to_string(i) + "/command", 10, callback);
    subs_consai_command_.push_back(sub_command);
  }

  RCLCPP_INFO(this->get_logger(), "Start");
}

void VsslCommandSender::callback_consai_command(const ConsaiCommand::SharedPtr msg, const int id)
{
  MoveVelocity * velocity = new MoveVelocity();
  velocity->set_forward(msg->velocity_x);
  velocity->set_left(msg->velocity_y);
  velocity->set_angular(msg->velocity_theta);

  RobotControl control;
  control.set_allocated_move_velocity(velocity);
  control.set_kick_speed(msg->kick_power);
  // control.set_dribbler_speed(msg->dribble_power);

  std::string output;
  control.SerializeToString(&output);
  senders_[id]->send(output);
}

}  // namespace consai_robot_controller

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(consai_robot_controller::VsslCommandSender)
