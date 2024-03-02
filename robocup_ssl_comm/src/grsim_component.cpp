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


#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "robocup_ssl_comm/grsim_component.hpp"
#include "robocup_ssl_msgs/grSim_Packet.pb.h"

using namespace std::chrono_literals;

namespace robocup_ssl_comm
{

using std::placeholders::_1;

GrSim::GrSim(const rclcpp::NodeOptions & options)
: Node("grsim", options)
{
  declare_parameter("robot_control_ip", "127.0.0.1");
  declare_parameter("robot_control_port", 20011);

  std::string robot_control_ip = get_parameter("robot_control_ip").as_string();
  int robot_control_port = get_parameter("robot_control_port").as_int();
  sender_ = std::make_unique<udp_sender::UDPSender>(robot_control_ip, robot_control_port);

  sub_commands_ = create_subscription<Commands>(
    "commands", 10, std::bind(&GrSim::callback_commands, this, _1));
  sub_replacement_ = create_subscription<Replacement>(
    "replacement", 10, std::bind(&GrSim::callback_replacement, this, _1));

  // timer_ = create_wall_timer(1s, std::bind(&GrSim::on_timer, this));
}

void GrSim::on_timer()
{
  RCLCPP_INFO(this->get_logger(), "Hello World!");
}

void GrSim::callback_commands(const Commands::SharedPtr msg)
{
  grSim_Commands * commands = new grSim_Commands();

  commands->set_timestamp(msg->timestamp);
  commands->set_isteamyellow(msg->isteamyellow);
  for (const auto & msg_robot_command : msg->robot_commands) {
    set_command(commands->add_robot_commands(), msg_robot_command);
  }

  grSim_Packet packet;
  packet.set_allocated_commands(commands);

  std::string output;
  packet.SerializeToString(&output);
  sender_->send(output);
}

void GrSim::callback_replacement(const Replacement::SharedPtr msg)
{
  grSim_Replacement * replacement = new grSim_Replacement();

  if (msg->ball.size() > 0) {
    grSim_BallReplacement * ball = new grSim_BallReplacement();
    auto msg_ball = msg->ball[0];
    if (msg_ball.x.size() > 0) {ball->set_x(msg_ball.x[0]);}
    if (msg_ball.y.size() > 0) {ball->set_y(msg_ball.y[0]);}
    if (msg_ball.vx.size() > 0) {ball->set_vx(msg_ball.vx[0]);}
    if (msg_ball.vy.size() > 0) {ball->set_vy(msg_ball.vy[0]);}
    replacement->set_allocated_ball(ball);
  }

  for (const auto & msg_robot : msg->robots) {
    set_robot_replacement(replacement->add_robots(), msg_robot);
  }

  grSim_Packet packet;
  packet.set_allocated_replacement(replacement);

  std::string output;
  packet.SerializeToString(&output);
  sender_->send(output);
}

void GrSim::set_command(grSim_Robot_Command * robot_command, const RobotCommand & msg_robot_command)
{
  robot_command->set_id(msg_robot_command.id);
  robot_command->set_kickspeedx(msg_robot_command.kickspeedx);
  robot_command->set_kickspeedz(msg_robot_command.kickspeedz);
  robot_command->set_veltangent(msg_robot_command.veltangent);
  robot_command->set_velnormal(msg_robot_command.velnormal);
  robot_command->set_velangular(msg_robot_command.velangular);
  robot_command->set_spinner(msg_robot_command.spinner);
  robot_command->set_wheelsspeed(msg_robot_command.wheelsspeed);
  if (msg_robot_command.wheel1.size() > 0) {
    robot_command->set_wheel1(msg_robot_command.wheel1[0]);
  }
  if (msg_robot_command.wheel2.size() > 0) {
    robot_command->set_wheel2(msg_robot_command.wheel2[0]);
  }
  if (msg_robot_command.wheel3.size() > 0) {
    robot_command->set_wheel3(msg_robot_command.wheel3[0]);
  }
  if (msg_robot_command.wheel4.size() > 0) {
    robot_command->set_wheel4(msg_robot_command.wheel4[0]);
  }
}

void GrSim::set_robot_replacement(
  grSim_RobotReplacement * robot_replacement,
  const RobotReplacement & msg_robot_replacement)
{
  robot_replacement->set_x(msg_robot_replacement.x);
  robot_replacement->set_y(msg_robot_replacement.y);
  robot_replacement->set_dir(msg_robot_replacement.dir);
  robot_replacement->set_id(msg_robot_replacement.id);
  robot_replacement->set_yellowteam(msg_robot_replacement.yellowteam);
  if (msg_robot_replacement.turnon.size() > 0) {
    robot_replacement->set_turnon(msg_robot_replacement.turnon[0]);
  }
}

}  // namespace robocup_ssl_comm

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(robocup_ssl_comm::GrSim)
