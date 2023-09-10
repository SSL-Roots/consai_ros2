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

  declare_parameter("udp_address", "192.168.0.10");
  declare_parameter("udp_port", 10003);

  sender_ = std::make_unique<udp_sender::UDPSender>(
    get_parameter("udp_address").get_value<std::string>(),
    get_parameter("udp_port").get_value<int>());

  for (int i = 0; i < 16; i++) {
    auto sub_command = create_subscription<ConsaiCommand>(
      "robot" + std::to_string(i) + "/command",
      10, std::bind(&VsslCommandSender::callback_consai_command_, this, _1));
    subs_consai_command_.push_back(sub_command);
  }

  timer_ = create_wall_timer(10ms, std::bind(&VsslCommandSender::on_timer, this));

  std::cout<<"Start" << std::endl;
}

void VsslCommandSender::on_timer()
{
  // consai用のロボットコマンドをgrSim用のコマンドに変換してpublishする

  if (consai_commands_.size() == 0) {
    return;
  }

  for (auto it = consai_commands_.begin(); it != consai_commands_.end(); ) {
    // 0番ロボットのコマンドをVSSLロボットに送信する
    if ( (*it)->robot_id == 0 ) {
      MoveVelocity * velocity = new MoveVelocity();
      velocity->set_forward((*it)->velocity_x);
      velocity->set_left((*it)->velocity_y);
      velocity->set_angular((*it)->velocity_theta);

      RobotControl control;
      control.set_allocated_move_velocity(velocity);
      control.set_kick_speed((*it)->kick_power);

      std::string output;
      control.SerializeToString(&output);
      sender_->send(output);
    }

    it = consai_commands_.erase(it);
  }
}

void VsslCommandSender::callback_consai_command_(const ConsaiCommand::SharedPtr msg)
{
  consai_commands_.push_back(msg);
  // バッファの肥大化を防ぐ
  if (consai_commands_.size() > 200) {
    consai_commands_.clear();
  }
}

}  // namespace consai_robot_controller

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(consai_robot_controller::VsslCommandSender)
