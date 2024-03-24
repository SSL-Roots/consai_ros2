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

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "consai_frootspi_msgs/msg/robot_command.hpp"
#include "consai_msgs/action/robot_control.hpp"
#include "consai_msgs/msg/goal_pose.hpp"
#include "consai_msgs/msg/goal_poses.hpp"
#include "consai_msgs/msg/named_targets.hpp"
#include "consai_msgs/msg/parsed_referee.hpp"
#include "consai_msgs/msg/state2_d.hpp"
#include "consai_robot_controller/field_info_parser.hpp"
#include "consai_robot_controller/visibility_control.h"
#include "consai_robot_controller/visualization_data_handler.hpp"
#include "consai_robot_controller/detection_extractor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robocup_ssl_msgs/msg/geometry_data.hpp"
#include "robocup_ssl_msgs/msg/referee.hpp"
#include "robocup_ssl_msgs/msg/tracked_frame.hpp"
#include "robocup_ssl_msgs/msg/tracked_robot.hpp"

namespace consai_robot_controller
{
using GoalPose = consai_msgs::msg::GoalPose;
using GoalPoses = consai_msgs::msg::GoalPoses;
using NamedTargets = consai_msgs::msg::NamedTargets;
using State = consai_msgs::msg::State2D;
using RobotCommand = consai_frootspi_msgs::msg::RobotCommand;
using RobotControl = consai_msgs::action::RobotControl;
using GoalHandleRobotControl = rclcpp_action::ServerGoalHandle<RobotControl>;
using GeometryData = robocup_ssl_msgs::msg::GeometryData;
using ParsedReferee = consai_msgs::msg::ParsedReferee;
using Referee = robocup_ssl_msgs::msg::Referee;
using TrackedFrame = robocup_ssl_msgs::msg::TrackedFrame;
using TrackedRobot = robocup_ssl_msgs::msg::TrackedRobot;
using GoalPosesMap = std::map<unsigned int, GoalPose>;

class Controller : public rclcpp::Node
{
public:
  CONSAI_ROBOT_CONTROLLER_PUBLIC
  explicit Controller(const rclcpp::NodeOptions & options);

protected:
  void on_timer_pub_control_command(const unsigned int robot_id);
  void on_timer_pub_goal_poses();

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const RobotControl::Goal> goal,
    const unsigned int robot_id);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleRobotControl> goal_handle,
    const unsigned int robot_id);
  void handle_accepted(
    std::shared_ptr<GoalHandleRobotControl> goal_handle,
    const unsigned int robot_id);
  State limit_world_velocity(
    const State & velocity, const double & max_velocity_xy,
    const double & max_velocity_theta) const;
  State limit_world_acceleration(
    const State & velocity, const State & last_velocity,
    const rclcpp::Duration & dt) const;
  bool arrived(const TrackedRobot & my_robot, const State & goal_pose);
  bool switch_to_stop_control_mode(
    const unsigned int robot_id, const bool success, const std::string & error_msg);

  std::vector<rclcpp::Publisher<RobotCommand>::SharedPtr> pub_command_;
  std::vector<rclcpp_action::Server<RobotControl>::SharedPtr> server_control_;
  std::vector<rclcpp::Time> last_update_time_;
  std::vector<rclcpp::TimerBase::SharedPtr> timer_pub_control_command_;
  std::vector<bool> control_enable_;
  std::vector<bool> need_response_;
  std::vector<std::shared_ptr<GoalHandleRobotControl>> goal_handle_;
  std::vector<State> last_world_vel_;

  std::shared_ptr<consai_robot_controller::FieldInfoParser> parser_;
  std::shared_ptr<parser::DetectionExtractor> detection_extractor_;

  rclcpp::Subscription<TrackedFrame>::SharedPtr sub_detection_tracked_;
  rclcpp::Subscription<GeometryData>::SharedPtr sub_geometry_;
  rclcpp::Subscription<NamedTargets>::SharedPtr sub_named_targets_;
  rclcpp::Subscription<Referee>::SharedPtr sub_referee_;
  rclcpp::Subscription<ParsedReferee>::SharedPtr sub_parsed_referee_;
  rclcpp::Publisher<GoalPoses>::SharedPtr pub_goal_poses_;
  rclcpp::Publisher<GoalPoses>::SharedPtr pub_final_goal_poses_;
  rclcpp::TimerBase::SharedPtr timer_pub_goal_poses_;
  std::shared_ptr<VisualizationDataHandler> vis_data_handler_;
  GoalPosesMap goal_poses_map_;
  GoalPosesMap final_goal_poses_map_;
  bool team_is_yellow_;
  rclcpp::Clock steady_clock_;
};

}  // namespace consai_robot_controller

#endif  // CONSAI_ROBOT_CONTROLLER__CONTROLLER_COMPONENT_HPP_
