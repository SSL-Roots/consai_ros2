// Copyright 2025 Roots
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

#include "consai_robot_controller/controller_unit.hpp"
#include "consai_robot_controller/tools/control_tools.hpp"
#include "consai_tools/geometry_tools.hpp"

namespace consai_robot_controller
{

void ControllerUnit::set_robot_command_publisher(
  rclcpp::Node::SharedPtr node, const std::string & topic_name)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
    // .best_effort()  // データの損失を許容する
    .durability_volatile();  // データの保存を行わない

  pub_command_ = node->create_publisher<RobotCommand>(topic_name, qos);
}


void ControllerUnit::move_to_desired_pose(
  const Pose2D & goal_pose,
  const TrackedRobot & my_robot,
  const double kick_power, const double dribble_power,
  std::optional<double> limit_vel_xy
)
{
  auto control_params = desired_control_params_;

  // 最低速度が指定されている場合は、リミットを更新する
  if (limit_vel_xy) {
    if (*limit_vel_xy < control_params.soft_limit_velocity_xy) {
      control_params.soft_limit_velocity_xy = *limit_vel_xy;
    }
  }

  // パラメータが更新された場合、
  // またはgoal_poseが変わった場合に軌道を再生成する
  bool need_regenerate_trajectory = false;
  if (control_params != locomotion_controller_.getControlParams()) {
    locomotion_controller_.setControlParams(control_params);
    need_regenerate_trajectory = true;

  } else if (goal_pose != locomotion_controller_.getGoal()) {
    need_regenerate_trajectory = true;
  }

  if (need_regenerate_trajectory) {
    locomotion_controller_.moveToPose(goal_pose);
    RCLCPP_DEBUG(rclcpp::get_logger("ControllerUnit"), "Regenerate trajectory via updated control params");
  }

  // 制御の実行
  auto [world_vel_, controller_state] = this->locomotion_controller_.run(
    State2D(
      Pose2D(my_robot.pos.x, my_robot.pos.y, my_robot.orientation),
      Velocity2D(my_robot.vel[0].x, my_robot.vel[0].y, my_robot.vel_angular[0])
    )
  );
  world_vel_.theta = calculate_angular_velocity(world_vel_, goal_pose, my_robot);

  auto command_msg = std::make_unique<RobotCommand>();
  command_msg->robot_id = robot_id_;
  command_msg->team_is_yellow = team_is_yellow_;

  // ワールド座標系でのxy速度をロボット座標系に変換
  const auto robot_theta = my_robot.orientation;
  command_msg->velocity_x = std::cos(robot_theta) * world_vel_.x + std::sin(robot_theta) * world_vel_.y;
  command_msg->velocity_y = -std::sin(robot_theta) * world_vel_.x + std::cos(robot_theta) * world_vel_.y;
  command_msg->velocity_theta = world_vel_.theta;

  // キックパワー、ドリブルパワーをセット
  command_msg->kick_power = kick_power;
  command_msg->dribble_power = dribble_power;

  pub_command_->publish(std::move(command_msg));
}

void ControllerUnit::publish_stop_command(void)
{
  auto stop_command_msg = std::make_unique<RobotCommand>();
  stop_command_msg->robot_id = robot_id_;
  stop_command_msg->team_is_yellow = team_is_yellow_;
  pub_command_->publish(std::move(stop_command_msg));
}

void ControllerUnit::set_control_params(const ControlParams & control_params)
{
  desired_control_params_ = control_params;
}

double ControllerUnit::calculate_angular_velocity(
    const Velocity2D & desired_velocity, const Pose2D & goal_pose, const TrackedRobot & my_robot)
{
  // 直進を安定させるため、xy速度が大きいときは、角度制御をしない
  const auto vel_norm = std::hypot(desired_velocity.x, desired_velocity.y);
  if (vel_norm > 1.5) {
    return 0.0;
  }

  // sin関数を用いた角速度制御
  const auto diff_theta = geometry_tools::normalize_theta(
    goal_pose.theta - my_robot.orientation);
  const auto angular_velocity = control_tools::angular_velocity_contol_sin(
    diff_theta,
    desired_control_params_.control_a_theta * desired_control_params_.hard_limit_velocity_theta);
  return angular_velocity;
}


}  // namespace consai_robot_controller