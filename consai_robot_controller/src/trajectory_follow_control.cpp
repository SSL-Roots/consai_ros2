// Copyright 2023 Roots
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

#include "consai_robot_controller/trajectory_follow_control.hpp"

#include <cstdint>

#include "consai_tools/geometry_tools.hpp"

// TrajectoryFollowController クラスの定義
TrajectoryFollowController::TrajectoryFollowController()
{
  this->state_ = ControllerState::INITIALIZED;
  this->tracked_time_ = 0;
  this->kp_linear_ = 1.0;
  this->trajectory_ = nullptr;
}

TrajectoryFollowController::TrajectoryFollowController(
  _Float64 kp_linear, _Float64 kd_linear,
  _Float64 kp_angular_, _Float64 kd_angular,
  double dt)
{
  this->state_ = ControllerState::INITIALIZED;
  this->tracked_time_ = 0;
  this->kp_linear_ = kp_linear;
  this->kd_linear_ = kd_linear;
  this->kp_angular_ = kp_angular_;
  this->kd_angular_ = kd_angular;
  this->dt_ = dt;
  this->latest_target_state_ = State2D();
  this->trajectory_ = nullptr;
}


void TrajectoryFollowController::initialize(std::shared_ptr<BangBangTrajectory3D> trajectory)
{
  this->trajectory_ = trajectory;
  this->state_ = ControllerState::INITIALIZED;
  this->latest_target_state_ = State2D();
  this->tracked_time_ = 0;
}

std::pair<Velocity2D, TrajectoryFollowController::ControllerState> TrajectoryFollowController::run(
  const State2D & current_state)
{
  /**
   * 軌道追従の制御を行う制御器
   * 現在位置と次ステップの目標軌道位置との差分を元にPID制御を行う。
   * また、フィードフォワード項として、現ステップと次ステップの目標位置の差分(=理想の速度)を使用する。
   */
  Velocity2D output = Velocity2D();

  // 状態の判定
  ControllerState state = ControllerState::RUNNING;

  if (this->state_ == ControllerState::INITIALIZED) {
    this->state_ = ControllerState::RUNNING;
  }

  // 今ステップの目標位置の取得
  Pose2D target_pose = this->trajectory_->get_pose(this->tracked_time_ + this->dt_);

  // 今ステップの目標速度の取得
  Velocity2D target_velocity = this->trajectory_->get_velocity(this->tracked_time_ + this->dt_);

  // x方向の制御
  ControllerOutput x_output = this->controlLinear(
    FF_AND_P, current_state.pose.x, target_pose.x, target_velocity.x);
  output.x = x_output.output;

  // y方向の制御
  ControllerOutput y_output = this->controlLinear(
    FF_AND_P, current_state.pose.y, target_pose.y, target_velocity.y);
  output.y = y_output.output;

  // 角度の制御
  ControllerOutput theta_output = this->controlAngular(
    current_state.pose.theta, target_pose.theta, target_velocity.theta);
  output.theta = theta_output.output;

  // 時間の更新
  this->tracked_time_ += this->dt_;

  this->latest_target_state_ = State2D(target_pose, target_velocity);

  // 終了判定
  if (this->tracked_time_ >= this->trajectory_->get_total_time()) {
    state = ControllerState::COMPLETE;
  }

  return std::make_pair(output, state);
}

TrajectoryFollowController::ControllerOutput TrajectoryFollowController::controlLinear(
  ControllerMode mode, double current_position,
  double target_position, double target_velocity)
{
  ControllerOutput output;

  if (mode == FF_AND_P) {
    double error = target_position - current_position;
    output.ff_term = target_velocity;
    output.p_term = kp_linear_ * error;
    output.output = output.ff_term + output.p_term;
    return output;
  } else if (mode == P) {
    double error = target_position - current_position;
    output.ff_term = 0.0;
    output.p_term = kp_linear_ * error;
    output.output = output.p_term;
    return output;
  } else {
    output.ff_term = 0.0;
    output.p_term = 0.0;
    output.output = 0.0;
    return output;
  }
}

TrajectoryFollowController::ControllerOutput TrajectoryFollowController::controlAngular(
  double current_position, double target_position,
  double target_velocity)
{
  double error = geometry_tools::normalize_theta(target_position - current_position);

  ControllerOutput output;
  output.ff_term = target_velocity;
  output.p_term = kp_angular_ * error;
  output.output = output.ff_term + output.p_term;

  // double output = kp_angular_ * error;

  return output;
}


// TrajectoryVisualizer クラスの定義
consai_visualizer_msgs::msg::Objects TrajectoryVisualizer::createObjectsFromTrajectory(
  BangBangTrajectory2D & trajectory)
{
  consai_visualizer_msgs::msg::Objects objects;

  objects.layer = "trajectory";
  objects.sub_layer = "trajectory";

  double dt = 0.1;
  for (double t = 0; t < trajectory.get_total_time(); t += dt) {
    consai_visualizer_msgs::msg::ShapeLine line;

    line.p1.x = trajectory.get_position(t).x;
    line.p1.y = trajectory.get_position(t).y;
    line.p2.x = trajectory.get_position(t + dt).x;
    line.p2.y = trajectory.get_position(t + dt).y;
    line.size = 1;
    line.color.red = 0.0;
    line.color.green = 0.0;
    line.color.blue = 1.0;
    line.color.alpha = 1.0;
    objects.lines.push_back(line);
  }

  return objects;
}
