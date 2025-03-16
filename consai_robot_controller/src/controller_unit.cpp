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


void ControllerUnit::publish_robot_command(
  const State & goal_pose,
  const TrackedRobot & my_robot,
  const double kick_power, const double dribble_power,
  std::optional<double> limit_vel_xy
)
{
  if (limit_vel_xy) {
    if (*limit_vel_xy < desired_control_params_.soft_limit_velocity_xy) {
      desired_control_params_.soft_limit_velocity_xy = *limit_vel_xy;
    }
  }

  // パラメータが更新された場合は、ロボットの制御器に反映する
  if (desired_control_params_ != locomotion_controller_.getControlParams()) {
    // TODO: ControlParamをそのまま参照するように書き換える
    locomotion_controller_.setParameters(
      desired_control_params_.p_gain_xy, 
      desired_control_params_.d_gain_xy,
      desired_control_params_.p_gain_theta,
      desired_control_params_.d_gain_theta,
      desired_control_params_.hard_limit_velocity_xy,
      desired_control_params_.soft_limit_velocity_xy,
      desired_control_params_.hard_limit_velocity_theta,
      desired_control_params_.soft_limit_velocity_theta,
      desired_control_params_.hard_limit_acceleration_xy,
      desired_control_params_.soft_limit_acceleration_xy,
      desired_control_params_.hard_limit_acceleration_theta,
      desired_control_params_.soft_limit_acceleration_theta
    );
    locomotion_controller_.setControlParams(desired_control_params_);

    // 軌道の再生成
    locomotion_controller_.moveToPose(
      Pose2D(goal_pose.x, goal_pose.y, goal_pose.theta)
    );
  }

  // 前回の目標値と今回が異なる場合にのみmoveToPoseを呼び出す
  Pose2D current_goal_pose = locomotion_controller_.getGoal();
  if (goal_pose.x != current_goal_pose.x || goal_pose.y != current_goal_pose.y ||
    goal_pose.theta != current_goal_pose.theta)
  {
    locomotion_controller_.moveToPose(
      Pose2D(goal_pose.x, goal_pose.y, goal_pose.theta)
    );
  }

  // 制御の実行
  auto [output_vel, controller_state] = this->locomotion_controller_.run(
    State2D(
      Pose2D(my_robot.pos.x, my_robot.pos.y, my_robot.orientation),
      Velocity2D(my_robot.vel[0].x, my_robot.vel[0].y, my_robot.vel_angular[0])
    )
  );

  // TODO: パラメータを動的に更新する
  const auto control_a_theta = 0.5;
  const auto hard_limit_vel_theta = 2.0 * M_PI;

  State world_vel;

  world_vel.x = output_vel.x;
  world_vel.y = output_vel.y;
  world_vel.theta = output_vel.theta;

  // sin関数を用いた角速度制御
  double diff_theta = geometry_tools::normalize_theta(
    goal_pose.theta - my_robot.orientation);
  world_vel.theta = control_tools::angular_velocity_contol_sin(
    diff_theta,
    control_a_theta *
    hard_limit_vel_theta);

  // 直進を安定させるため、xy速度が大きいときは、角度制御をしない
  const auto vel_norm = std::hypot(world_vel.x, world_vel.y);
  if (vel_norm > 1.5) {
    world_vel.theta = 0.0;
  }

  auto command_msg = std::make_unique<RobotCommand>();
  command_msg->robot_id = robot_id_;
  command_msg->team_is_yellow = team_is_yellow_;

  // ワールド座標系でのxy速度をロボット座標系に変換
  const auto robot_theta = my_robot.orientation;
  command_msg->velocity_x = std::cos(robot_theta) * world_vel.x + std::sin(
    robot_theta) * world_vel.y;
  command_msg->velocity_y = -std::sin(robot_theta) * world_vel.x + std::cos(
    robot_theta) * world_vel.y;
  command_msg->velocity_theta = world_vel.theta;

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


}  // namespace consai_robot_controller