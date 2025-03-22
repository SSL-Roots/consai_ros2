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

  steady_clock_ = rclcpp::Clock(RCL_STEADY_TIME);

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
  if (robot_control_map_[robot_id]->stop) {
    controller_unit_[robot_id].publish_stop_command();
    return;
  }

  if (!parser_->is_parsable(robot_control_map_[robot_id])) {
    controller_unit_[robot_id].publish_stop_command();
    return;
  }

  // 制御するロボットの情報を得る
  // ロボットの情報が存在しなければ制御を終える
  TrackedRobot my_robot;
  if (!detection_extractor_->extract_robot(robot_id, team_is_yellow_, my_robot)) {
    std::string error_msg = "Failed to extract ID:" + std::to_string(robot_id) +
      " robot from detection_tracked msg.";
    RCLCPP_WARN(this->get_logger(), error_msg.c_str());

    controller_unit_[robot_id].publish_stop_command();
    robot_control_map_[robot_id]->stop = true;
    return;
  }

  // 目標値を取得する
  // 目標値を取得できなければ速度0を目標値とする
  State goal_pose;
  State destination;
  double kick_power = 0.0;
  double dribble_power = 0.0;

  const auto current_time = steady_clock_.now();
  if (!parser_->parse_goal(
      robot_control_map_[robot_id], my_robot, goal_pose, destination, kick_power,
      dribble_power))
  {
    RCLCPP_WARN(this->get_logger(), "Failed to parse goal of robot_id:%d", robot_id);
    controller_unit_[robot_id].publish_stop_command();
    return;
  }

  // field_info_parserの衝突回避を無効化する場合は、下記の行をコメントアウトすること
  goal_pose = parser_->modify_goal_pose_to_avoid_obstacles(
    robot_control_map_[robot_id], my_robot, goal_pose, destination);


  // 最大速度が上書きされていたらそちらの値を使う
  std::optional<double> limit_vel_xy = std::nullopt;
  if (robot_control_map_[robot_id]->max_velocity_xy.size() > 0) {
    limit_vel_xy = robot_control_map_[robot_id]->max_velocity_xy[0];
  }

  controller_unit_[robot_id].move_to_desired_pose(
    Pose2D(goal_pose), my_robot, kick_power, dribble_power, limit_vel_xy);

  controller_unit_[robot_id].publish_debug_data(my_robot);

  // ビジュアライズ用に、目標姿勢と最終目標姿勢を出力する
  GoalPose goal_pose_msg;
  goal_pose_msg.robot_id = robot_id;
  goal_pose_msg.team_is_yellow = team_is_yellow_;
  goal_pose_msg.pose = goal_pose;
  goal_poses_map_[robot_id] = goal_pose_msg;

  GoalPose destination_msg;
  destination_msg.robot_id = robot_id;
  destination_msg.team_is_yellow = team_is_yellow_;
  destination_msg.pose = destination;
  destinations_map_[robot_id] = destination_msg;
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

    robot_control_map_[i] = std::make_shared<RobotControlMsg>();
    auto robot_control_callback = [this](
      const RobotControlMsg::SharedPtr msg, const unsigned int robot_id) {
        this->robot_control_map_[robot_id] = msg;
      };
    // Can not use auto. Ref: https://github.com/ros2/rclcpp/issues/273
    std::function<void(const RobotControlMsg::SharedPtr msg)> fcn = std::bind(
      robot_control_callback, _1, i);

    std::string name_space = team_color + std::to_string(i);
    sub_robot_control_.push_back(
      create_subscription<RobotControlMsg>(
        name_space + "/control", 10, fcn));

    // bindでは関数を宣言できなかったので、ラムダ式を使用する
    // Ref: https://github.com/ros2/rclcpp/issues/273#issuecomment-263826519
    timer_pub_control_command_.push_back(
      create_wall_timer(
        control_loop_cycle_, [this, robot_id = i]() {this->on_timer_pub_control_command(robot_id);}
      )
    );
  }
}

}  // namespace consai_robot_controller

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(consai_robot_controller::Controller)
