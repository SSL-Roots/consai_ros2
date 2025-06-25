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

#include <memory>
#include <string>
#include <utility>

#include "consai_robot_controller/grsim_command_converter.hpp"
#include "nlohmann/json.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace consai_robot_controller
{

GrSimCommandConverter::GrSimCommandConverter(const rclcpp::NodeOptions & options)
: Node("controller", options)
{
  pub_grsim_commands_ = create_publisher<GrSimCommands>("/commands", 10);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliable();

  auto callback_param = [this](const std_msgs::msg::String::SharedPtr msg)
    {
      try {
        nlohmann::json json_data = nlohmann::json::parse(msg->data);
        gen_command_subscribers(json_data["robots"]["num_of_ids"]);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Error: %s", e.what());
      }
    };

  sub_consai_param_rule_ = create_subscription<std_msgs::msg::String>(
    "consai_param/rule", qos, callback_param);

  timer_ = create_wall_timer(10ms, std::bind(&GrSimCommandConverter::on_timer, this));
}

void GrSimCommandConverter::on_timer()
{
  // consai用のロボットコマンドをgrSim用のコマンドに変換してpublishする

  if (consai_commands_.size() == 0) {
    return;
  }

  auto commands_msg = std::make_unique<GrSimCommands>();

  bool team_is_yellow;
  for (auto it = consai_commands_.begin(); it != consai_commands_.end(); ) {
    GrSimRobotCommand robot_command;
    team_is_yellow = (*it)->team_is_yellow;
    robot_command.id = (*it)->robot_id;
    robot_command.veltangent = (*it)->velocity_x;
    robot_command.velnormal = (*it)->velocity_y;
    robot_command.velangular = (*it)->velocity_theta;

    if ((*it)->dribble_power > 0.0001) {
      robot_command.spinner = true;
    }

    robot_command.kickspeedx = (*it)->kick_power;

    if ((*it)->chip_kick) {
      robot_command.kickspeedz = (*it)->kick_power;
    } else {
      robot_command.kickspeedz = 0.0f;
    }

    commands_msg->robot_commands.push_back(robot_command);
    it = consai_commands_.erase(it);
  }

  commands_msg->isteamyellow = team_is_yellow;
  pub_grsim_commands_->publish(std::move(commands_msg));
}

void GrSimCommandConverter::gen_command_subscribers(const unsigned int num)
{
  // subs_consai_command_の大きさがnumより小さければ、サブスクライバを追加する
  if (subs_consai_command_.size() >= num) {
    return;
  }

  for (auto i = subs_consai_command_.size(); i < num; i++) {
    auto sub_command = create_subscription<ConsaiCommand>(
      "robot" + std::to_string(i) + "/command",
      10, std::bind(&GrSimCommandConverter::callback_consai_command_, this, _1));
    subs_consai_command_.push_back(sub_command);
  }
}

void GrSimCommandConverter::callback_consai_command_(const ConsaiCommand::SharedPtr msg)
{
  consai_commands_.push_back(msg);
  // バッファの肥大化を防ぐ
  if (consai_commands_.size() > 200) {
    consai_commands_.clear();
  }
}

}  // namespace consai_robot_controller

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(consai_robot_controller::GrSimCommandConverter)
