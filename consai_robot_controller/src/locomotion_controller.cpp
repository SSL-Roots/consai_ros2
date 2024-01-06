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

#include "consai_robot_controller/locomotion_controller.hpp"

#include <algorithm>

#include "consai_robot_controller/trajectory/bangbangtrajectory3d.hpp"


LocomotionController::LocomotionController(
  _Float64 kp_xy, _Float64 kd_xy, _Float64 kp_theta,
  _Float64 kd_theta, double dt, double max_linear_velocity,
  double max_angular_velocity,
  double max_linear_acceleration,
  double max_angular_acceleration)
{
  this->trajectory_follow_controller_ = TrajectoryFollowController(
    kp_xy, kd_xy, kp_theta, kd_theta,
    dt);
  this->target_velocity_ = Velocity2D(0, 0, 0);
  this->output_velocity_ = Velocity2D(0, 0, 0);
  this->state_ = INITIALIZED;
  this->kp_xy = kp_xy;
  this->kp_theta = kp_theta;
  this->dt_ = dt;
  this->max_linear_velocity_ = max_linear_velocity;
  this->max_angular_velocity_ = max_angular_velocity;
  this->max_linear_acceleration_ = max_linear_acceleration;
  this->max_angular_acceleration_ = max_angular_acceleration;
}

LocomotionController::ControllerState LocomotionController::moveConstantVelocity(
  const Velocity2D & velocity)
{
  // 一定速度での移動を指示するメソッドの実装
  target_velocity_ = Velocity2D(velocity.x, velocity.y, velocity.theta);

  state_ = RUNNING_CONSTANT_VELOCITY;
  return state_;
}

LocomotionController::ControllerState LocomotionController::moveToPose(
  const Pose2D & goal_pose,
  const Pose2D & current_pose)
{
  // 特定のポーズへの移動を指示するメソッドの実装

  // 軌道生成を行う
  BangBangTrajectory3D trajectory;

  Pose2D s0, s1;
  Velocity2D v0;
  if (this->state_ == INITIALIZED || this->state_ == RUNNING_CONSTANT_VELOCITY) {
    // 位置追従制御に切り替わるタイミングでは、現在の位置と速度を初期値として軌道生成を行う
    s0 = Pose2D(current_pose.x, current_pose.y, current_pose.theta);
    s1 = Pose2D(goal_pose.x, goal_pose.y, goal_pose.theta);
    v0 =
      Velocity2D(this->output_velocity_.x, this->output_velocity_.y, this->output_velocity_.theta);
    // TODO(tilt_silvie): ロボットの現在速度を使うように変える
  } else {
    // 位置追従制御中に新たな目標位置が与えられた場合は、直前の目標位置と速度を初期値として軌道生成を行う
    s0 = Pose2D(
      this->trajectory_follow_controller_.latest_target_state_.pose.x,
      this->trajectory_follow_controller_.latest_target_state_.pose.y,
      this->trajectory_follow_controller_.latest_target_state_.pose.theta);
    s1 = Pose2D(goal_pose.x, goal_pose.y, goal_pose.theta);
    v0 = Velocity2D(
      this->trajectory_follow_controller_.latest_target_state_.velocity.x,
      this->trajectory_follow_controller_.latest_target_state_.velocity.y,
      this->trajectory_follow_controller_.latest_target_state_.velocity.theta);
  }

  trajectory.generate(
    s0, s1, v0, this->max_linear_velocity_ * 0.8,
    this->max_angular_velocity_ * 0.8, this->max_linear_acceleration_ * 0.8,
    this->max_angular_acceleration_ * 0.8, 0.1);

  trajectory_follow_controller_.initialize(std::make_shared<BangBangTrajectory3D>(trajectory));

  state_ = RUNNING_FOLLOW_TRAJECTORY;
  return state_;
}

std::pair<Velocity2D, LocomotionController::ControllerState> LocomotionController::run(
  const State2D & current_state)
{
  // 現在の状態から次のステップの速度と状態を計算するメソッドの実装
  Velocity2D output_velocity;

  switch (this->state_) {
    case RUNNING_CONSTANT_VELOCITY:
      // 一定速度での移動を継続
      output_velocity = target_velocity_;
      break;

    case GENERATING_TRAJECTORY:
      // 軌道生成を実行
      break;

    case RUNNING_FOLLOW_TRAJECTORY:
      // 軌道追従制御を実行
      output_velocity = runFollowTrajectory(current_state);
      break;

    case COMPLETE:
      // 完了
      output_velocity = Velocity2D(0, 0, 0);
      break;

    default:
      // 未定義の状態
      output_velocity = Velocity2D(0, 0, 0);
      break;
  }

  Velocity2D acc_limited_velocity = limitAcceleration(
    output_velocity, this->output_velocity_,
    this->dt_);
  Velocity2D vel_limited_velocity = limitVelocity(acc_limited_velocity);

  this->output_velocity_ = vel_limited_velocity;
  return std::make_pair(this->output_velocity_, state_);
}

LocomotionController::ControllerState LocomotionController::getState()
{
  // コントローラの状態を取得するメソッドの実装
  return state_;
}

State2D LocomotionController::getCurrentTargetState()
{
  return this->trajectory_follow_controller_.latest_target_state_;
}

/**
 * Private
*/
Velocity2D LocomotionController::limitAcceleration(
  const Velocity2D & velocity, const Velocity2D & last_velocity,
  const double & dt) const
{
  // ワールド座標系のロボット加速度に制限を掛ける
  Velocity2D acc;   // TODO(tilt_silvie): 型を修正する
  acc.x = (velocity.x - last_velocity.x) / dt;
  acc.y = (velocity.y - last_velocity.y) / dt;
  acc.theta = (velocity.theta - last_velocity.theta) / dt;

  auto acc_norm = std::hypot(acc.x, acc.y);
  auto acc_ratio = acc_norm / this->max_linear_acceleration_;

  if (acc_ratio > 1.0) {
    acc.x /= acc_ratio;
    acc.y /= acc_ratio;
  }
  acc.theta = std::clamp(acc.theta, -this->max_angular_acceleration_, max_angular_acceleration_);

  Velocity2D modified_velocity = velocity;
  modified_velocity.x = last_velocity.x + acc.x * dt;
  modified_velocity.y = last_velocity.y + acc.y * dt;
  modified_velocity.theta = last_velocity.theta + acc.theta * dt;

  return modified_velocity;
}

Velocity2D LocomotionController::limitVelocity(
  const Velocity2D & velocity) const
{
  // ワールド座標系のロボット速度に制限を掛ける
  auto velocity_norm = std::hypot(velocity.x, velocity.y);
  auto velocity_ratio = velocity_norm / this->max_linear_velocity_;

  Velocity2D modified_velocity = velocity;
  if (velocity_ratio > 1.0) {
    modified_velocity.x /= velocity_ratio;
    modified_velocity.y /= velocity_ratio;
  }
  modified_velocity.theta = std::clamp(
    velocity.theta, -this->max_angular_velocity_,
    this->max_angular_velocity_);

  return modified_velocity;
}


Velocity2D LocomotionController::runFollowTrajectory(const State2D & current_state)
{
  auto control_output = trajectory_follow_controller_.run(current_state);
  Velocity2D output = control_output.first;
  if (control_output.second == TrajectoryFollowController::ControllerState::COMPLETE) {
    this->state_ = COMPLETE;
  }

  return output;
}
