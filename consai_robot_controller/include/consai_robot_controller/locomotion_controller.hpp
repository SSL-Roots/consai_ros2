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

#pragma once

#include <utility>
#include "rclcpp/rclcpp.hpp"
#include "consai_robot_controller/trajectory_follow_control.hpp"
#include "consai_robot_controller/trajectory_generator.hpp"


class LocomotionController
{
public:
  enum ControllerState
  {
    INITIALIZED,
    RUNNING_CONSTANT_VELOCITY,
    GENERATING_TRAJECTORY,
    RUNNING_FOLLOW_TRAJECTORY,
    COMPLETE,
    FAILED,
  };

  LocomotionController();

  LocomotionController(int robot_id_for_debug, double dt);

  LocomotionController(
    int robot_id_for_debug, _Float64 kp_xy, _Float64 kd_xy, _Float64 kp_theta, _Float64 kd_theta,
    double delayfactor_sec, double dt, 
    double hard_limit_linear_velocity, double soft_limit_linear_velocity,
    double hard_limit_angular_velocity, double soft_limit_angular_velocity,
    double hard_limit_linear_acceleration, double soft_limit_linear_acceleration,
    double hard_limit_angular_acceleration, double soft_limit_angular_acceleration);

  ControllerState moveConstantVelocity(const Velocity2D & velocity);
  ControllerState moveToPose(const Pose2D & goal_pose, const State2D & current_state);
  ControllerState halt();
  std::pair<Velocity2D, ControllerState> run(const State2D & current_state);
  ControllerState getState();
  State2D getCurrentTargetState();
  Pose2D getGoal();
  void setParameters(
    _Float64 kp_xy, _Float64 kd_xy, _Float64 kp_theta, _Float64 kd_theta, 
    double hard_limit_linear_velocity, double soft_limit_linear_velocity,
    double hard_limit_angular_velocity, double soft_limit_angular_velocity,
    double hard_limit_linear_acceleration, double soft_limit_linear_acceleration,
    double hard_limit_angular_acceleration, double soft_limit_angular_acceleration);  

  double getHardLimitLinearVelocity();
  double getHardLimitAngularVelocity();
  double getHardLimitLinearAcceleration();
  double getHardLimitAngularAcceleration();

private:
  TrajectoryFollowController trajectory_follow_controller_;
  Velocity2D target_velocity_;
  Velocity2D output_velocity_;
  Pose2D goal_pose_;
  ControllerState state_;

  _Float64 kp_xy_;
  _Float64  ki_xy_;
  _Float64  kd_xy_;
  _Float64 kp_theta_;  // [rad]
  _Float64  ki_theta_;
  _Float64  kd_theta_;
  double delayfactor_sec_;
  double dt_;           // 制御周期
  double hard_limit_linear_velocity_;        // [m/s]
  double soft_limit_linear_velocity_;        // [m/s]
  double hard_limit_angular_velocity_;       // [rad/s]
  double soft_limit_angular_velocity_;       // [rad/s]
  double hard_limit_linear_acceleration_;    // [m/s^2]
  double soft_limit_linear_acceleration_;    // [m/s^2]
  double hard_limit_angular_acceleration_;   // [rad/s^2]
  double soft_limit_angular_acceleration_;   // [rad/s^2]
  int robot_id_for_debug_;

  void initializeTrajectoryFollowController(std::shared_ptr<BangBangTrajectory3D> trajectory);
  void generateTrajectory(const Pose2D & goal_pose, const State2D & current_state);
  Velocity2D runFollowTrajectory(const State2D & current_state);
  Velocity2D limitAcceleration(
    const Velocity2D & velocity, const Velocity2D & last_velocity,
    const double & dt) const;
  Velocity2D limitVelocity(
    const Velocity2D & velocity) const;
};
