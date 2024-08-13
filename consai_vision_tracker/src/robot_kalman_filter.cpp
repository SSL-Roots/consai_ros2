// Copyright 2024 Roots
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

#include "consai_vision_tracker/robot_kalman_filter.hpp"
#include "robocup_ssl_msgs/msg/vector2.hpp"

#include <iostream>

namespace consai_vision_tracker
{

using Vector2d = Eigen::Vector2d;
using Matrix2d = Eigen::Matrix2d;
using Matrix63d = Eigen::Matrix<double, 6, 3>;
using SSLVector2 = robocup_ssl_msgs::msg::Vector2;

RobotKalmanFilter::RobotKalmanFilter(
  const int team_color, const int id,
  const double dt, const double lifetime,
  const double visibility_increase_rate,
  const double outlier_time_limit)
:DT_(dt),
  VISIBILITY_CONTROL_VALUE_(dt / lifetime),
  VISIBILITY_INCREASE_RATE_(visibility_increase_rate),
  OUTLIER_COUNT_THRESHOLD_(outlier_time_limit / dt)
{
  robot_id_.team_color = team_color;
  robot_id_.id = id;

  // State transition matrix
  F_ = Matrix6d::Identity();
  F_(0, 3) = DT_;
  F_(1, 4) = DT_;
  F_(2, 5) = DT_;

  // Observation matrix
  H_ = Matrix36d::Zero();
  H_(0, 0) = 1.0;
  H_(1, 1) = 1.0;
  H_(2, 2) = 1.0;

  // Process noise covariance matrix
  // Observation noise covariance matrix
  update_noise_covariance_matrix(
    50.0,  // xy m/s^2
    30.0,  // theta rad/s^2
    0.05,  // xy m
    0.05  // theta rad
  );

  // State vector
  // x_ = (x, y, theta, vx, vy, vtheta)
  // Error covariance matrix
  reset_x_and_p(Vector3d::Zero());
}

void RobotKalmanFilter::push_back_observation(const DetectionRobot & robot)
{
  // Empty orientation data means that the data is not available
  if (robot.orientation.size() == 0) {
    return;
  }

  TrackedRobot observation;
  observation.pos.x = robot.x * 0.001;  // mm to meters
  observation.pos.y = robot.y * 0.001;  // mm to meters
  observation.orientation = robot.orientation[0];
  robot_observations_.push_back(observation);
}

TrackedRobot RobotKalmanFilter::update(void)
{
  Vector6d x_pred;
  Matrix6d P_pred;

  auto prediction = [&, this](void) {
      x_pred = F_ * x_;
      x_pred(2) = normalize_angle(x_pred(2));
      P_pred = F_ * P_ * F_.transpose() + Q_;
    };

  auto calc_innovation = [&](const Vector3d & z) {
      Vector3d innovation = z - H_ * x_pred;
      innovation(2) = normalize_angle(z(2) - (H_ * x_pred)(2));
      return innovation;
    };

  auto calc_chi_square = [&](const Vector3d & z) {
      Matrix3d S = H_ * P_pred * H_.transpose() + R_;
      Vector3d innovation = calc_innovation(z);

    // Use x and y only
      Matrix2d S_xy = S.block<2, 2>(0, 0);
      Vector2d innovation_xy = innovation.block<2, 1>(0, 0);

      double chi_square = innovation_xy.transpose() * S_xy.inverse() * innovation_xy;
      return chi_square;
    };

  // Prediction step
  prediction();

  // Remove outliers
  for (auto it = robot_observations_.begin(); it != robot_observations_.end(); ) {
    const Vector3d z(it->pos.x, it->pos.y, it->orientation);
    const double chi_square = calc_chi_square(z);

    if (!is_outlier(chi_square)) {
      outlier_count_ = 0;
      it++;
      continue;
    }

    outlier_count_++;
    if (outlier_count_ < OUTLIER_COUNT_THRESHOLD_) {
      it = robot_observations_.erase(it);
    } else {
      reset_x_and_p(z);
      prediction();  // Use new state vector
      it++;
    }
  }

  // Make observation
  Vector3d z(x_[0], x_[1], x_[2]);
  if (robot_observations_.empty()) {
    visibility_ = std::max(0.0, visibility_ - VISIBILITY_CONTROL_VALUE_);
  } else {
    z = make_observation();
    visibility_ = std::min(1.0,
        visibility_ + VISIBILITY_INCREASE_RATE_ * VISIBILITY_CONTROL_VALUE_);
    robot_observations_.clear();
  }

  // Calculate innovation
  Vector3d innovation = calc_innovation(z);
  // Calculate innovation covariance matrix
  Matrix3d S = H_ * P_pred * H_.transpose() + R_;

  // Update step
  Matrix63d K = P_pred * H_.transpose() * S.inverse();
  x_ = x_pred + K * innovation;
  x_(2) = normalize_angle(x_(2));
  P_ = (Matrix6d::Identity() - K * H_) * P_pred;

  return get_estimation();
}

TrackedRobot RobotKalmanFilter::get_estimation(void) const
{
  TrackedRobot estimation;
  estimation.robot_id = robot_id_;
  estimation.pos.x = x_(0);
  estimation.pos.y = x_(1);
  estimation.orientation = x_(2);

  SSLVector2 vel;
  vel.x = x_(3);
  vel.y = x_(4);
  estimation.vel.push_back(vel);
  estimation.vel_angular.push_back(x_(5));
  estimation.visibility.push_back(visibility_);

  return estimation;
}

void RobotKalmanFilter::update_noise_covariance_matrix(
  const double q_max_acc_xy, const double q_max_acc_theta,
  const double r_pos_stddev_xy, const double r_pos_stddev_theta)
{
  // Process noise covariance matrix
  auto gen_Q = [this](const double q_max_acc_xy, const double q_max_acc_theta) {
    // Process noise depends on the maximum acceleration
      const double sigma_vel_xy = q_max_acc_xy * DT_;
      const double sigma_vel_theta = q_max_acc_theta * DT_;
      const double sigma_pos_xy = 0.5 * q_max_acc_xy * DT_ * DT_;
      const double sigma_pos_theta = 0.5 * q_max_acc_theta * DT_ * DT_;

      const double var_vel_xy = std::pow(sigma_vel_xy, 2);
      const double var_vel_theta = std::pow(sigma_vel_theta, 2);
      const double var_pos_xy = std::pow(sigma_pos_xy, 2);
      const double var_pos_theta = std::pow(sigma_pos_theta, 2);

      const double sigma_vel_pos_xy = sigma_vel_xy * sigma_pos_xy;
      const double sigma_vel_pos_theta = sigma_vel_theta * sigma_pos_theta;

      Matrix6d Q = Matrix6d::Zero();
      Q(0, 0) = var_pos_xy;
      Q(1, 1) = var_pos_xy;
      Q(2, 2) = var_pos_theta;
      Q(3, 3) = var_vel_xy;
      Q(4, 4) = var_vel_xy;
      Q(5, 5) = var_vel_theta;

      Q(0, 3) = sigma_vel_pos_xy;
      Q(1, 4) = sigma_vel_pos_xy;
      Q(2, 5) = sigma_vel_pos_theta;
      Q(3, 0) = sigma_vel_pos_xy;
      Q(4, 1) = sigma_vel_pos_xy;
      Q(5, 2) = sigma_vel_pos_theta;
      return Q;
    };
  Q_ = gen_Q(q_max_acc_xy, q_max_acc_theta);

  // Observation noise covariance matrix
  auto gen_R = [&](const double std_dev_xy, const double std_dev_theta) {
      const double var_xy = std_dev_xy * std_dev_xy;
      const double var_theta = std_dev_theta * std_dev_theta;
      return Eigen::DiagonalMatrix<double, 3>(var_xy, var_xy, var_theta);
    };
  R_ = gen_R(r_pos_stddev_xy, r_pos_stddev_theta);
}

RobotLocalVelocity RobotKalmanFilter::calc_local_velocity(void) const
{
  auto local_velocity = RobotLocalVelocity();
  local_velocity.robot_id = robot_id_;

  const auto estimation = get_estimation();
  const auto theta = estimation.orientation;
  const auto world_vel = estimation.vel[0];

  auto local_vel = world_vel;
  local_vel.x = world_vel.x * std::cos(theta) + world_vel.y * std::sin(theta);
  local_vel.y = -(world_vel.x * std::sin(theta) - world_vel.y * std::cos(theta));
  local_velocity.velocity.x = local_vel.x;
  local_velocity.velocity.y = local_vel.y;
  local_velocity.norm = std::sqrt(std::pow(local_vel.x, 2) + std::pow(local_vel.y, 2));
  local_velocity.velocity.theta = estimation.vel_angular[0];

  return local_velocity;
}

Vector3d RobotKalmanFilter::make_observation(void) const
{
  if (robot_observations_.empty()) {
    return Vector3d(x_[0], x_[1], x_[2]);
  }

  Vector3d mean_observation(0.0, 0.0, 0.0);
  double sum_cos = 0.0;
  double sum_sin = 0.0;
  for (const auto & observation : robot_observations_) {
    mean_observation(0) += observation.pos.x;
    mean_observation(1) += observation.pos.y;
    sum_cos += std::cos(observation.orientation);
    sum_sin += std::sin(observation.orientation);
  }
  mean_observation(0) /= robot_observations_.size();
  mean_observation(1) /= robot_observations_.size();
  sum_cos /= robot_observations_.size();
  sum_sin /= robot_observations_.size();
  mean_observation(2) = std::atan2(sum_sin, sum_cos);

  return mean_observation;
}

bool RobotKalmanFilter::is_outlier(const double chi_squared_value) const
{
  constexpr double THRESHOLD = 5.991;  // 2 degrees of freedom, 0.05 significance level

  if (chi_squared_value > THRESHOLD) {
    return true;
  }
  return false;
}

void RobotKalmanFilter::reset_x_and_p(const Vector3d & observation)
{
  x_ << observation[0], observation[1], observation[2], 0.0, 0.0, 0.0;
  P_ = Matrix6d::Identity() * 10.0;
}

double RobotKalmanFilter::normalize_angle(const double angle) const
{
  // Normalize angle to -pi ~ pi
  double result = angle;
  while (result > M_PI) {result -= 2.0 * M_PI;}
  while (result < -M_PI) {result += 2.0 * M_PI;}
  return result;
}

} // namespace consai_vision_tracker
