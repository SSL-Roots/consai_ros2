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

#include <memory>
#include <utility>
#include "rclcpp/rclcpp.hpp"
#include "consai_robot_controller/control_params.hpp"
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

  LocomotionController(
    int robot_id_for_debug,
    double dt,
    double delayfactor_sec = 0.0);

  ControllerState moveConstantVelocity(const Velocity2D & velocity);
  ControllerState moveToPose(const Pose2D & goal_pose);
  std::pair<Velocity2D, ControllerState> run(const State2D & current_state);
  ControllerState getState() const;
  State2D getCurrentTargetState() const;
  Pose2D getGoal() const;

  consai_robot_controller::ControlParams getControlParams() const
  {
    return control_params_;
  }
  void setControlParams(const consai_robot_controller::ControlParams & control_params)
  {
    control_params_ = control_params;
  }

private:
  TrajectoryFollowController trajectory_follow_controller_;
  Velocity2D target_velocity_;
  Velocity2D output_velocity_;
  Pose2D goal_pose_;
  ControllerState state_;

  const int robot_id_for_debug_;
  const double dt_;  // 制御周期
  const double delayfactor_sec_;
  consai_robot_controller::ControlParams control_params_;

  void initializeTrajectoryFollowController(std::shared_ptr<BangBangTrajectory3D> trajectory);
  void generateTrajectory(const Pose2D & goal_pose, const State2D & current_state);
  Velocity2D runFollowTrajectory(const State2D & current_state);
  Velocity2D limitAcceleration(
    const Velocity2D & velocity, const Velocity2D & last_velocity,
    const double & dt) const;
  Velocity2D limitVelocity(
    const Velocity2D & velocity) const;
};
