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

#include "robocup_ssl_comm/game_controller_component.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robocup_ssl_msgs/msg/point.hpp"

using namespace std::chrono_literals;

namespace robocup_ssl_comm
{

GameController::GameController(const rclcpp::NodeOptions & options)
: Node("game_controller", options)
{
  declare_parameter("multicast_address", "224.5.23.1");
  declare_parameter("multicast_port", 10003);
  receiver_ = std::make_unique<multicast::MulticastReceiver>(
    get_parameter("multicast_address").get_value<std::string>(), get_parameter(
      "multicast_port").get_value<int>());
  pub_referee_ = create_publisher<robocup_ssl_msgs::msg::Referee>("referee", 10);
  timer_ = create_wall_timer(25ms, std::bind(&GameController::on_timer, this));
}

void GameController::on_timer()
{
  while (receiver_->available()) {
    std::vector<char> buf(2048);
    const size_t size = receiver_->receive(buf);

    if (size > 0) {
      Referee packet;
      packet.ParseFromString(std::string(buf.begin(), buf.end()));

      auto referee_msg = std::make_unique<robocup_ssl_msgs::msg::Referee>();

      referee_msg->packet_timestamp = packet.packet_timestamp();
      referee_msg->stage = packet.stage();
      if (packet.has_stage_time_left()) {
        referee_msg->stage_time_left.push_back(packet.stage_time_left());
      }
      referee_msg->command = packet.command();
      referee_msg->command_counter = packet.command_counter();
      referee_msg->command_timestamp = packet.command_timestamp();

      referee_msg->yellow = parse_team_info(packet.yellow());
      referee_msg->blue = parse_team_info(packet.blue());
      if (packet.has_designated_position()) {
        robocup_ssl_msgs::msg::Point point;
        point.x = packet.designated_position().x();
        point.y = packet.designated_position().y();
        referee_msg->designated_position.push_back(point);
      }
      if (packet.has_blue_team_on_positive_half()) {
        referee_msg->blue_team_on_positive_half.push_back(packet.blue_team_on_positive_half());
      }
      if (packet.has_next_command()) {
        referee_msg->next_command.push_back(packet.next_command());
      }
      if (packet.has_current_action_time_remaining()) {
        referee_msg->current_action_time_remaining.push_back(
          packet.current_action_time_remaining());
      }

      pub_referee_->publish(std::move(referee_msg));
    }
  }
}

robocup_ssl_msgs::msg::TeamInfo GameController::parse_team_info(const Referee_TeamInfo & team_info)
{
  robocup_ssl_msgs::msg::TeamInfo parsed_team_info;

  parsed_team_info.name = team_info.name();
  parsed_team_info.score = team_info.score();
  parsed_team_info.red_cards = team_info.red_cards();
  for (auto time : team_info.yellow_card_times()) {
    parsed_team_info.yellow_card_times.push_back(time);
  }
  parsed_team_info.yellow_cards = team_info.yellow_cards();
  parsed_team_info.timeouts = team_info.timeouts();
  parsed_team_info.timeout_time = team_info.timeout_time();
  parsed_team_info.goalkeeper = team_info.goalkeeper();
  if (team_info.has_foul_counter()) {
    parsed_team_info.foul_counter.push_back(team_info.foul_counter());
  }
  if (team_info.has_ball_placement_failures()) {
    parsed_team_info.ball_placement_failures.push_back(team_info.ball_placement_failures());
  }
  if (team_info.has_can_place_ball()) {
    parsed_team_info.can_place_ball.push_back(team_info.can_place_ball());
  }
  if (team_info.has_max_allowed_bots()) {
    parsed_team_info.max_allowed_bots.push_back(team_info.max_allowed_bots());
  }
  if (team_info.has_bot_substitution_intent()) {
    parsed_team_info.bot_substitution_intent.push_back(team_info.bot_substitution_intent());
  }
  if (team_info.has_ball_placement_failures_reached()) {
    parsed_team_info.ball_placement_failures_reached.push_back(
      team_info.ball_placement_failures_reached());
  }

  return parsed_team_info;
}

}  // namespace robocup_ssl_comm

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(robocup_ssl_comm::GameController)
