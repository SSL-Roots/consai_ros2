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


#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "consai_robot_controller/controller_component.hpp"
#include "consai_robot_controller/tools/control_tools.hpp"
#include "consai_robot_controller/control_params.hpp"
#include "consai_tools/geometry_tools.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"

namespace consai_robot_controller
{

namespace tools = geometry_tools;
namespace ctools = control_tools;

using namespace std::placeholders;
using namespace std::chrono_literals;

Controller::Controller(const rclcpp::NodeOptions & options)
: Node("controller", options)
{
  declare_parameter("team_is_yellow", false);
  declare_parameter("invert", false);

  const auto visibility_threshold = 0.01;
  detection_extractor_ = std::make_shared<parser::DetectionExtractor>(visibility_threshold);
  parser_ = std::make_shared<FieldInfoParser>(
    get_parameter("team_is_yellow").get_value<bool>(),
    get_parameter("invert").get_value<bool>(),
    detection_extractor_);

  team_is_yellow_ = get_parameter("team_is_yellow").get_value<bool>();
  RCLCPP_INFO(this->get_logger(), "is yellow:%d", team_is_yellow_);

  clock_ = rclcpp::Clock(RCL_ROS_TIME);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliable();

  auto callback_param_rule = [this](const std_msgs::msg::String::SharedPtr msg)
    {
      try {
        nlohmann::json json_data = nlohmann::json::parse(msg->data);
        gen_pubs_and_subs(json_data["robots"]["num_of_ids"]);
        parser_->set_consai_param_rule(json_data);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Param rule callback error: %s", e.what());
      }
    };

  sub_consai_param_rule_ = create_subscription<std_msgs::msg::String>(
    "consai_param/rule", qos, callback_param_rule);

  auto callback_param_control = [this](const std_msgs::msg::String::SharedPtr msg)
    {
      try {
        nlohmann::json json_data = nlohmann::json::parse(msg->data);
        ControlParams control_params;
        control_params.hard_limit_acceleration_xy = json_data["hard_limits"]["acceleration_xy"];
        control_params.hard_limit_acceleration_theta =
          json_data["hard_limits"]["acceleration_theta"];
        control_params.hard_limit_velocity_xy = json_data["hard_limits"]["velocity_xy"];
        control_params.hard_limit_velocity_theta = json_data["hard_limits"]["velocity_theta"];

        control_params.soft_limit_acceleration_xy = json_data["soft_limits"]["acceleration_xy"];
        control_params.soft_limit_acceleration_theta =
          json_data["soft_limits"]["acceleration_theta"];
        control_params.soft_limit_velocity_xy = json_data["soft_limits"]["velocity_xy"];
        control_params.soft_limit_velocity_theta = json_data["soft_limits"]["velocity_theta"];

        control_params.p_gain_xy = json_data["gains"]["p_xy"];
        control_params.p_gain_theta = json_data["gains"]["p_theta"];
        control_params.d_gain_xy = json_data["gains"]["d_xy"];
        control_params.d_gain_theta = json_data["gains"]["d_theta"];
        control_params.control_a_theta = json_data["gains"]["a_theta"];

        for (auto & unit : controller_unit_) {
          unit.set_control_params(control_params);
        }
        RCLCPP_INFO(get_logger(), "Update control parameters");
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Param control callback error: %s", e.what());
      }
    };

  sub_consai_param_control_ = create_subscription<std_msgs::msg::String>(
    "consai_param/control", qos, callback_param_control);

  auto callback_motion_command_array = [this](const MotionCommandArray::SharedPtr msg)
    {
      for (const auto & command : msg->commands) {
        motion_command_map_[command.robot_id] = command;
        motion_command_map_[command.robot_id].header.stamp = clock_.now();
      }
    };
  sub_motion_command_array_ = create_subscription<MotionCommandArray>(
    "motion_commands", qos, callback_motion_command_array);

  pub_goal_poses_ = create_publisher<GoalPoses>("goal_poses", 10);
  pub_destinations_ = create_publisher<GoalPoses>("destinations", 10);
  vis_data_handler_ = std::make_shared<VisualizationDataHandler>(
    create_publisher<VisualizerObjects>(
      "visualizer_objects", rclcpp::SensorDataQoS()));
  timer_pub_goal_poses_ =
    create_wall_timer(
    this->control_loop_cycle_, std::bind(&Controller::on_timer_pub_goal_poses, this)
    );

  parser_->set_subscriptions(this);
}

void Controller::on_timer_pub_control_command(const unsigned int robot_id)
{
  if (controller_unit_.size() <= robot_id) {
    return;
  }

  TrackedRobot my_robot;
  if (!detection_extractor_->extract_robot(robot_id, team_is_yellow_, my_robot)) {
    controller_unit_[robot_id].publish_stop_command();
    return;
  }

  // 古い動作コマンド（control_msg）と新しいコマンド（motion_command）の両方に対応する
  MotionCommand command;

  std::optional<double> limit_vel_xy = std::nullopt;
  if (auto navi = calc_navi_data_from_control_msg(my_robot)) {
    navi->motion_command.desired_pose = parser_->modify_goal_pose_to_avoid_obstacles(
      robot_control_map_.at(robot_id),
      my_robot,
      navi->motion_command.desired_pose,
      navi->destination);


    command = navi->motion_command;
    // 最大速度が上書きされていたらそちらの値を使う
    if (robot_control_map_[robot_id]->max_velocity_xy.size() > 0) {
      limit_vel_xy = robot_control_map_[robot_id]->max_velocity_xy[0];
    }

    controller_unit_[robot_id].move_to_desired_pose(
      Pose2D(command.desired_pose),
      my_robot,
      command.kick_power,
      command.dribble_power,
      command.chip_kick,
      limit_vel_xy);

    destinations_map_[robot_id].pose = navi->destination;  // デバッグ用
    goal_poses_map_[robot_id].pose = command.desired_pose;  // デバッグ用
  } else if (auto it = motion_command_map_.find(robot_id); it != motion_command_map_.end()) {
    const auto & motion_command = it->second;

    // 古いコマンドは無視する
    if (clock_.now() - motion_command.header.stamp > 1s) {
      controller_unit_[robot_id].publish_stop_command();
      return;
    }

    command = motion_command;

    if (command.mode == MotionCommand::MODE_NAVI) {
      // desired_poseまでNavigation経由で移動する場合の処理

      destinations_map_[robot_id].pose = command.desired_pose;  // デバッグ用

      // 衝突回避
      command.desired_pose = parser_->modify_goal_pose_to_avoid_obstacles(
        my_robot, command.desired_pose, command.navi_options);

      if (command.desired_velocity.x > 0) {
        limit_vel_xy = command.desired_velocity.x;
      }

      controller_unit_[robot_id].move_to_desired_pose(
        Pose2D(command.desired_pose),
        my_robot,
        command.kick_power,
        command.dribble_power,
        command.chip_kick,
        limit_vel_xy);

      goal_poses_map_[robot_id].pose = command.desired_pose;  // デバッグ用

    } else if (command.mode == MotionCommand::MODE_DIRECT_POSE) {
      // desired_poseまでNavigation経由で移動する場合の処理

      destinations_map_[robot_id].pose = command.desired_pose;  // デバッグ用

      if (command.desired_velocity.x > 0) {
        limit_vel_xy = command.desired_velocity.x;
      }

      // 目標位置まで移動
      controller_unit_[robot_id].move_to_desired_pose(
        Pose2D(command.desired_pose),
        my_robot,
        command.kick_power,
        command.dribble_power,
        command.chip_kick,
        limit_vel_xy);

      goal_poses_map_[robot_id].pose = command.desired_pose;  // デバッグ用

    } else if (command.mode == MotionCommand::MODE_DIRECT_VELOCITY) {
      controller_unit_[robot_id].publish_velocity_command(
        command.desired_velocity,
        command.kick_power,
        command.dribble_power,
        command.chip_kick);

      // 描画が分かりにくくならないように、ロボットの位置で上書きする
      State robot_pos;
      robot_pos.x = my_robot.pos.x;
      robot_pos.y = my_robot.pos.y;
      robot_pos.theta = my_robot.orientation;
      destinations_map_[robot_id].pose = robot_pos;  // デバッグ用
      goal_poses_map_[robot_id].pose = robot_pos;  // デバッグ用
    } else {
      RCLCPP_WARN(get_logger(), "Unknown motion command mode: %d", command.mode);
      controller_unit_[robot_id].publish_stop_command();
      return;
    }
  } else {
    controller_unit_[robot_id].publish_stop_command();
    return;
  }

  controller_unit_[robot_id].publish_debug_data(my_robot);
}

void Controller::on_timer_pub_goal_poses()
{
  // 目標位置・姿勢を描画情報として出力する
  auto destinations_msg = std::make_unique<GoalPoses>();
  for (const auto & robot_id : detection_extractor_->active_robot_id_list(team_is_yellow_)) {
    TrackedRobot my_robot;
    if (!detection_extractor_->extract_robot(robot_id, team_is_yellow_, my_robot)) {
      continue;
    }

    if (goal_poses_map_.count(robot_id) > 0 && destinations_map_.count(robot_id) > 0) {
      vis_data_handler_->append_vis_goal(
        my_robot, goal_poses_map_[robot_id], destinations_map_[robot_id]);
      destinations_msg->poses.push_back(destinations_map_[robot_id]);
    }
  }

  vis_data_handler_->publish_and_reset_vis_goal();
  pub_destinations_->publish(std::move(destinations_msg));
}

void Controller::gen_pubs_and_subs(const unsigned int num)
{
  if (controller_unit_.size() >= num) {
    return;
  }

  std::string team_color = "blue";
  if (team_is_yellow_) {
    team_color = "yellow";
  }

  const auto dt = control_loop_cycle_.count() / 1000.0;

  for (auto i = controller_unit_.size(); i < num; i++) {
    controller_unit_.push_back(ControllerUnit(i, team_is_yellow_, dt));
    controller_unit_[i].set_robot_command_publisher(
      this->shared_from_this(), "robot" + std::to_string(i) + "/command");
    controller_unit_[i].set_debug_publishers(this->shared_from_this());

    const std::string name_space = team_color + std::to_string(i);

    robot_control_map_[i] = std::make_shared<RobotControlMsg>();
    sub_robot_control_.push_back(
      create_subscription<RobotControlMsg>(
        name_space + "/control", 10,
        [this, i](const RobotControlMsg::SharedPtr msg) {
          this->robot_control_map_[i] = msg;
        }));

    // bindでは関数を宣言できなかったので、ラムダ式を使用する
    // Ref: https://github.com/ros2/rclcpp/issues/273#issuecomment-263826519
    timer_pub_control_command_.push_back(
      create_wall_timer(
        control_loop_cycle_, [this, robot_id = i]() {this->on_timer_pub_control_command(robot_id);}
      )
    );

    GoalPose msg;
    msg.robot_id = i;
    msg.team_is_yellow = team_is_yellow_;
    goal_poses_map_[i] = msg;
    destinations_map_[i] = msg;
  }
}

std::optional<NaviData> Controller::calc_navi_data_from_control_msg(
  const TrackedRobot & my_robot) const
{
  const auto robot_id = my_robot.robot_id.id;

  if (robot_control_map_.count(robot_id) == 0) {
    return std::nullopt;
  }

  if (robot_control_map_.at(robot_id)->stop) {
    return std::nullopt;
  }

  if (!parser_->is_parsable(robot_control_map_.at(robot_id))) {
    return std::nullopt;
  }

  NaviData navi;
  navi.motion_command.robot_id = robot_id;

  if (!parser_->parse_goal(
      robot_control_map_.at(robot_id),
      my_robot,
      navi.motion_command.desired_pose,
      navi.destination,
      navi.motion_command.kick_power,
      navi.motion_command.dribble_power))
  {
    RCLCPP_WARN(this->get_logger(), "Failed to parse goal of robot_id:%d", robot_id);
    return std::nullopt;
  }

  return navi;
}

}  // namespace consai_robot_controller

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(consai_robot_controller::Controller)
