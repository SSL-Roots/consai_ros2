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

#include "consai_frootspi_msgs/msg/robot_command.hpp"
#include "consai_msgs/msg/goal_pose.hpp"
#include "consai_msgs/msg/goal_poses.hpp"
#include "consai_msgs/msg/named_targets.hpp"
#include "consai_msgs/msg/parsed_referee.hpp"
#include "consai_msgs/msg/robot_control_msg.hpp"
#include "consai_msgs/msg/state2_d.hpp"
#include "consai_robot_controller/field_info_parser.hpp"
#include "consai_robot_controller/locomotion_controller.hpp"
#include "consai_robot_controller/trajectory_follow_control.hpp"
#include "consai_robot_controller/visibility_control.h"
#include "consai_robot_controller/visualization_data_handler.hpp"
#include "consai_robot_controller/detection_extractor.hpp"
#include "consai_robot_controller/obstacle/obstacle_observer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robocup_ssl_msgs/msg/referee.hpp"
#include "robocup_ssl_msgs/msg/tracked_frame.hpp"
#include "robocup_ssl_msgs/msg/tracked_robot.hpp"
#include "std_msgs/msg/string.hpp"

namespace consai_robot_controller
{
using GoalPose = consai_msgs::msg::GoalPose;
using GoalPoses = consai_msgs::msg::GoalPoses;
using NamedTargets = consai_msgs::msg::NamedTargets;
using State = consai_msgs::msg::State2D;
using RobotCommand = consai_frootspi_msgs::msg::RobotCommand;
using RobotControlMsg = consai_msgs::msg::RobotControlMsg;
using ParsedReferee = consai_msgs::msg::ParsedReferee;
using Referee = robocup_ssl_msgs::msg::Referee;
using TrackedFrame = robocup_ssl_msgs::msg::TrackedFrame;
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
  State limit_world_velocity(
    const State & velocity, const double & max_velocity_xy,
    const double & max_velocity_theta) const;
  State limit_world_acceleration(
    const State & velocity, const State & last_velocity,
    const rclcpp::Duration & dt) const;
  bool arrived(const TrackedRobot & my_robot, const State & goal_pose);
  bool publish_stop_command(const unsigned int robot_id);

  std::vector<rclcpp::Publisher<RobotCommand>::SharedPtr> pub_command_;
  std::vector<rclcpp::Subscription<RobotControlMsg>::SharedPtr> sub_robot_control_;
  std::vector<rclcpp::Time> last_update_time_;
  std::vector<rclcpp::TimerBase::SharedPtr> timer_pub_control_command_;
  std::vector<State> last_world_vel_;
  std::vector<LocomotionController> locomotion_controller_;

  std::vector<rclcpp::Publisher<State>::SharedPtr> pub_current_pose_;
  std::vector<rclcpp::Publisher<State>::SharedPtr> pub_current_vel_;
  std::vector<rclcpp::Publisher<State>::SharedPtr> pub_goal_pose_;
  std::vector<rclcpp::Publisher<State>::SharedPtr> pub_target_speed_world_;
  std::vector<rclcpp::Publisher<State>::SharedPtr> pub_control_output_;
  std::vector<rclcpp::Publisher<State>::SharedPtr> pub_control_output_ff_;
  std::vector<rclcpp::Publisher<State>::SharedPtr> pub_control_output_p_;

  std::shared_ptr<consai_robot_controller::FieldInfoParser> parser_;
  std::shared_ptr<parser::DetectionExtractor> detection_extractor_;
  std::shared_ptr<obstacle::ObstacleObserver> obstacle_observer_;

  rclcpp::Subscription<TrackedFrame>::SharedPtr sub_detection_tracked_;
  rclcpp::Subscription<NamedTargets>::SharedPtr sub_named_targets_;
  rclcpp::Subscription<Referee>::SharedPtr sub_referee_;
  rclcpp::Subscription<ParsedReferee>::SharedPtr sub_parsed_referee_;
  rclcpp::Publisher<GoalPoses>::SharedPtr pub_goal_poses_;
  rclcpp::Publisher<GoalPoses>::SharedPtr pub_destinations_;
  rclcpp::TimerBase::SharedPtr timer_pub_goal_poses_;
  std::shared_ptr<VisualizationDataHandler> vis_data_handler_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_consai_param_rule_;

  RobotControlMap robot_control_map_;
  GoalPosesMap goal_poses_map_;
  GoalPosesMap destinations_map_;
  bool team_is_yellow_;
  rclcpp::Clock steady_clock_;

  const std::chrono::milliseconds control_loop_cycle_ = std::chrono::milliseconds(10);
};

}  // namespace consai_robot_controller

#endif  // CONSAI_ROBOT_CONTROLLER__CONTROLLER_COMPONENT_HPP_
