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

#ifndef ROBOCUP_SSL_COMM__GAME_CONTROLLER_COMPONENT_HPP_
#define ROBOCUP_SSL_COMM__GAME_CONTROLLER_COMPONENT_HPP_

#include <memory>

#include "robocup_ssl_comm/visibility_control.h"
#include "robocup_ssl_msgs/ssl_gc_referee_message.pb.h"

#include "multicast.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robocup_ssl_msgs/msg/referee.hpp"
#include "robocup_ssl_msgs/msg/team_info.hpp"

namespace robocup_ssl_comm
{

class GameController : public rclcpp::Node
{
public:
  ROBOCUP_SSL_COMM_PUBLIC
  explicit GameController(const rclcpp::NodeOptions & options);

protected:
  void on_timer();

private:
  robocup_ssl_msgs::msg::TeamInfo parse_team_info(const Referee_TeamInfo & team_info);

  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<multicast::MulticastReceiver> receiver_;
  rclcpp::Publisher<robocup_ssl_msgs::msg::Referee>::SharedPtr pub_referee_;
};

}  // namespace robocup_ssl_comm

#endif  // ROBOCUP_SSL_COMM__GAME_CONTROLLER_COMPONENT_HPP_
