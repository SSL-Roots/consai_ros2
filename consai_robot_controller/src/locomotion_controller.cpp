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
  int robot_id_for_debug, double dt,
  double delayfactor_sec)
: robot_id_for_debug_(robot_id_for_debug), dt_(dt), delayfactor_sec_(delayfactor_sec)
{
  this->target_velocity_ = Velocity2D(0, 0, 0);
  this->output_velocity_ = Velocity2D(0, 0, 0);
  this->state_ = INITIALIZED;
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
  const Pose2D & goal_pose)
{
  // 特定のポーズへの移動を指示するメソッドの実装

  // 経路の連続性を担保するために、実際の current_stateを利用せず、前回の目標位置・速度を利用する
  this->generateTrajectory(goal_pose, this->trajectory_follow_controller_.latest_target_state_);
  this->goal_pose_ = goal_pose;
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

LocomotionController::ControllerState LocomotionController::getState() const
{
  // コントローラの状態を取得するメソッドの実装
  return state_;
}

/**
 * @brief 現在の軌道上の仮想の目標状態を取得する
*/
State2D LocomotionController::getCurrentTargetState() const
{
  return this->trajectory_follow_controller_.latest_target_state_;
}

/**
 * @brief 現在の最終目標位置を取得する
*/
Pose2D LocomotionController::getGoal() const
{
  return this->goal_pose_;
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
  auto acc_ratio = acc_norm / control_params_.hard_limit_acceleration_xy;

  if (acc_ratio > 1.0) {
    acc.x /= acc_ratio;
    acc.y /= acc_ratio;
  }
  acc.theta = std::clamp(
    acc.theta, -control_params_.hard_limit_acceleration_theta,
    control_params_.hard_limit_acceleration_theta);

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
  auto velocity_ratio = velocity_norm / control_params_.hard_limit_velocity_xy;

  Velocity2D modified_velocity = velocity;
  if (velocity_ratio > 1.0) {
    modified_velocity.x /= velocity_ratio;
    modified_velocity.y /= velocity_ratio;
  }
  modified_velocity.theta = std::clamp(
    velocity.theta, -control_params_.hard_limit_velocity_theta,
    control_params_.hard_limit_velocity_theta);

  return modified_velocity;
}

void LocomotionController::initializeTrajectoryFollowController(
  std::shared_ptr<BangBangTrajectory3D> trajectory)
{
  this->trajectory_follow_controller_ = TrajectoryFollowController(
    this->robot_id_for_debug_,
    control_params_.p_gain_xy,
    control_params_.d_gain_xy,
    control_params_.p_gain_theta,
    control_params_.d_gain_theta,
    this->delayfactor_sec_, this->dt_,
    trajectory);
}

void LocomotionController::generateTrajectory(
  const Pose2D & goal_pose, const State2D & current_state)
{
  // 軌道生成を行う
  BangBangTrajectory3D trajectory;

  Pose2D s0, s1;
  Velocity2D v0;

  s0 = Pose2D(current_state.pose.x, current_state.pose.y, current_state.pose.theta);
  s1 = Pose2D(goal_pose.x, goal_pose.y, goal_pose.theta);
  v0 =
    Velocity2D(current_state.velocity.x, current_state.velocity.y, current_state.velocity.theta);

  trajectory.generate(
    s0, s1, v0,
    control_params_.soft_limit_velocity_xy,
    control_params_.soft_limit_velocity_theta,
    control_params_.soft_limit_acceleration_xy,
    control_params_.soft_limit_acceleration_theta, 0.1);

  this->initializeTrajectoryFollowController(std::make_shared<BangBangTrajectory3D>(trajectory));
}

Velocity2D LocomotionController::runFollowTrajectory(const State2D & current_state)
{
  auto control_output = trajectory_follow_controller_.run(current_state);

  if (control_output.second == TrajectoryFollowController::ControllerState::FAILED) {
    // 軌道追従に失敗したときは再度軌道を生成し直して追従し直す
    this->generateTrajectory(
      this->goal_pose_,
      current_state);

    control_output = trajectory_follow_controller_.run(current_state);
  }

  Velocity2D output = control_output.first;
  if (control_output.second == TrajectoryFollowController::ControllerState::COMPLETE) {
    this->state_ = COMPLETE;
  }

  return output;
}
