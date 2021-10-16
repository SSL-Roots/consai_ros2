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

#include "consai_robot_controller/controller_component.hpp"

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace consai_robot_controller
{

Controller::Controller(const rclcpp::NodeOptions & options)
: Node("controller", options)
{
  declare_parameter("robot_id", 0);
  declare_parameter("team_is_yellow", false);

  robot_id_ = get_parameter("robot_id").get_value<int>();
  team_is_yellow_ = get_parameter("team_is_yellow").get_value<bool>();

  RCLCPP_INFO(this->get_logger(), "robot_id is %d, is yellow:%d",robot_id_, team_is_yellow_);

  std::string command_name_space = "blue";
  if(team_is_yellow_){
    command_name_space = "yellow";
  }
  command_name_space += std::to_string(robot_id_);
  pub_command_ = create_publisher<consai_msgs::msg::RobotCommand>(command_name_space + "/command", 10);
  timer_ = create_wall_timer(1s, std::bind(&Controller::on_timer, this));
}

void Controller::on_timer()
{
  RCLCPP_INFO(this->get_logger(), "Hello World!");
}

}  // namespace consai_robot_controller

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(consai_robot_controller::Controller)
