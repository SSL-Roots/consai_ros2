#pragma once

#include "rclcpp/rclcpp.hpp"
#include "consai_robot_controller/trajectory_follow_control.hpp"
#include "consai_robot_controller/trajectory_generator.hpp"


class LocomotionController {
public:
  enum ControllerState {
    INITIALIZED,
    RUNNING_CONSTANT_VELOCITY,
    GENERATING_TRAJECTORY,
    RUNNING_FOLLOW_TRAJECTORY,
    COMPLETE,
    FAILED,
  };

  LocomotionController(_Float64 kp_xy, _Float64 kp_theta, double dt, double max_linear_velocity, double max_angular_velocity, double max_linear_acceleration, double max_angular_acceleration);

  ControllerState moveConstantVelocity(const Velocity2D& velocity);
  ControllerState moveToPose(const Pose2D& goal_pose, const Pose2D& current_pose);
  ControllerState halt();
  std::pair<Velocity2D, ControllerState> run(const State2D& current_state);
  ControllerState getState();
  State2D getCurrentTargetState();

private:
  TrajectoryFollowController trajectory_follow_controller_;
  Velocity2D target_velocity_;
  Velocity2D output_velocity_;
  ControllerState state_;

  _Float64  kp_xy;
  // _Float64  ki_xy;
  // _Float64  kd_xy;
  _Float64  kp_theta; // [rad]
  // _Float64  ki_theta;
  // _Float64  kd_theta;
  double dt_;           // 制御周期
  double max_linear_velocity_;    // [m/s]
  double max_angular_velocity_; // [rad/s]
  double max_linear_acceleration_; // [m/s^2]
  double max_angular_acceleration_; // [rad/s^2]

  Velocity2D runFollowTrajectory(const State2D& current_state);
};