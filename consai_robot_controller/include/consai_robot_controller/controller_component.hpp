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

#ifndef CONSAI_ROBOT_CONTROLLER__CONTROLLER_COMPONENT_HPP_
#define CONSAI_ROBOT_CONTROLLER__CONTROLLER_COMPONENT_HPP_

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "consai_msgs/msg/goal_pose.hpp"
#include "consai_msgs/msg/goal_poses.hpp"
#include "consai_msgs/msg/robot_control_msg.hpp"
#include "consai_msgs/msg/state2_d.hpp"
#include "consai_robot_controller/controller_unit.hpp"
#include "consai_robot_controller/field_info_parser.hpp"
#include "consai_robot_controller/trajectory_follow_control.hpp"
#include "consai_robot_controller/visibility_control.h"
#include "consai_robot_controller/visualization_data_handler.hpp"
#include "consai_robot_controller/detection_extractor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robocup_ssl_msgs/msg/tracked_robot.hpp"
#include "std_msgs/msg/string.hpp"

namespace consai_robot_controller
{
using GoalPose = consai_msgs::msg::GoalPose;
using GoalPoses = consai_msgs::msg::GoalPoses;
using State = consai_msgs::msg::State2D;
using RobotControlMsg = consai_msgs::msg::RobotControlMsg;
using TrackedRobot = robocup_ssl_msgs::msg::TrackedRobot;
using GoalPosesMap = std::map<unsigned int, GoalPose>;
using RobotControlMap = std::map<unsigned int, RobotControlMsg::SharedPtr>;

class Controller : public rclcpp::Node
{
public:
  CONSAI_ROBOT_CONTROLLER_PUBLIC
  explicit Controller(const rclcpp::NodeOptions & options);

protected:
  void on_timer_pub_control_command(const unsigned int robot_id);
  void on_timer_pub_goal_poses();

private:
  void gen_pubs_and_subs(const unsigned int num);

  std::vector<ControllerUnit> controller_unit_;

  std::vector<rclcpp::Subscription<RobotControlMsg>::SharedPtr> sub_robot_control_;
  std::vector<rclcpp::TimerBase::SharedPtr> timer_pub_control_command_;

  std::shared_ptr<consai_robot_controller::FieldInfoParser> parser_;
  std::shared_ptr<parser::DetectionExtractor> detection_extractor_;

  rclcpp::Publisher<GoalPoses>::SharedPtr pub_goal_poses_;
  rclcpp::Publisher<GoalPoses>::SharedPtr pub_destinations_;
  rclcpp::TimerBase::SharedPtr timer_pub_goal_poses_;
  std::shared_ptr<VisualizationDataHandler> vis_data_handler_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_consai_param_rule_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_consai_param_control_;

  RobotControlMap robot_control_map_;
  GoalPosesMap goal_poses_map_;
  GoalPosesMap destinations_map_;
  bool team_is_yellow_;
  rclcpp::Clock steady_clock_;

  const std::chrono::milliseconds control_loop_cycle_ = std::chrono::milliseconds(10);
};

}  // namespace consai_robot_controller

#endif  // CONSAI_ROBOT_CONTROLLER__CONTROLLER_COMPONENT_HPP_
