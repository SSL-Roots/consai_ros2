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

using Matrix63d = Eigen::Matrix<double, 6, 3>;
using SSLVector2 = robocup_ssl_msgs::msg::Vector2;

RobotKalmanFilter::RobotKalmanFilter(
  const int team_color, const int id,
  const double dt, const double lifetime,
  const double visibility_increase_rate,
  const double outlier_time_limit) :
  DT_(dt),
  VISIBILITY_CONTROL_VALUE_(dt / lifetime),
  VISIBILITY_INCREASE_RATE_(visibility_increase_rate),
  OUTLIER_COUNT_THRESHOLD_(outlier_time_limit / dt)
{
  robot_id_.team_color = team_color;
  robot_id_.id = id;

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
    x_pred = predict_state();
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
    double chi_square = innovation.transpose() * S.inverse() * innovation;
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
    visibility_ = std::min(1.0, visibility_ + VISIBILITY_INCREASE_RATE_ * VISIBILITY_CONTROL_VALUE_);
    robot_observations_.clear();
  }

  // Calculate innovation
  Vector3d innovation = calc_innovation(z);
  // Calculate innovation covariance matrix
  Matrix3d S = H_ * P_pred * H_.transpose() + R_;

  // Update step
  Matrix63d K = P_pred * H_.transpose() * S.inverse();
  x_ = x_pred + K * innovation;
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

Vector6d RobotKalmanFilter::predict_state(void) const
{
  // TODO: Use input data to predict state
  Vector6d x_pred;

  const double x = x_(0);
  const double y = x_(1);
  const double theta = x_(2);
  const double vx = x_(3);
  const double vy = x_(4);
  const double omega = x_(5);

  if (fabs(omega) > OMEGA_ZERO_) {
    x_pred(0) = x + (vx / omega) * (std::sin(theta + omega * DT_) - std::sin(theta))
                  + (vy / omega) * (std::cos(theta) - std::cos(theta + omega * DT_));
    x_pred(1) = y + (vx / omega) * (std::cos(theta) - std::cos(theta + omega * DT_))
                  + (vy / omega) * (std::sin(theta + omega * DT_) - std::sin(theta));
  } else {
    x_pred(0) = x + vx * DT_;
    x_pred(1) = y + vy * DT_;
  }

  x_pred(2) = normalize_angle(theta + omega * DT_);
  x_pred(3) = vx;
  x_pred(4) = vy;
  x_pred(5) = omega;

  return x_pred;
}

Matrix6d RobotKalmanFilter::jacobian_F(void) const
{
  // TODO: Use input data to calculate jacobian matrix
  Matrix6d F = Matrix6d::Identity();

  const double theta = x_(2);
  const double vx = x_(3);
  const double vy = x_(4);
  const double omega = x_(5);

  if (fabs(omega) > OMEGA_ZERO_) {
    const double VX_O = vx / omega;
    const double VY_O = vy / omega;
    const double VX_OO = vx / (omega * omega);
    const double VY_OO = vy / (omega * omega);
    const double INV_O = 1.0 / omega;
    const double ALPHA_THETA = theta + omega * DT_;

    F(0, 2) = VX_O * (std::cos(ALPHA_THETA) - std::cos(theta))
                - VY_O * (std::sin(theta) - std::sin(ALPHA_THETA));  // d(x)/d(theta)
    F(0, 3) = INV_O * (std::sin(ALPHA_THETA) - std::sin(theta));  // d(x)/d(vx)
    F(0, 4) = INV_O * (std::cos(theta) - std::cos(ALPHA_THETA));  // d(x)/d(vy)
    F(0, 5) = -VX_OO * (std::sin(ALPHA_THETA) - std::sin(theta))
                + VX_O * DT_ * std::cos(ALPHA_THETA)
                - VY_OO * (std::cos(theta) - std::cos(ALPHA_THETA))
                + VY_O * DT_ * std::sin(ALPHA_THETA);  // d(x)/d(omega);

    F(1, 2) = VX_O - (std::sin(theta) - std::sin(ALPHA_THETA))
                + VY_O * (std::cos(ALPHA_THETA) - std::cos(theta));  // d(y)/d(theta)
    F(1, 3) = INV_O * (std::cos(theta) - std::cos(ALPHA_THETA));  // d(y)/d(vx)
    F(1, 4) = INV_O * (std::sin(ALPHA_THETA) - std::sin(theta));  // d(y)/d(vy)
    F(1, 5) = -VX_OO * (std::cos(theta) - std::cos(ALPHA_THETA))
                + VX_O * DT_ * std::sin(ALPHA_THETA)
                - VY_OO * (std::sin(ALPHA_THETA) - std::sin(theta))
                + VY_O * DT_ * std::cos(ALPHA_THETA);  // d(y)/d(omega);

    F(2, 5) = DT_; // d(theta)/d(omega)
  } else {
    F(0, 3) = DT_;
    F(1, 4) = DT_;
    F(2, 5) = DT_;
  }

  return F;
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
  // 3 degrees of freedom (x, y, theta), 0.05 significance level
  constexpr double THRESHOLD = 7.815;

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
  while (result > M_PI) result -= 2.0 * M_PI;
  while (result < -M_PI) result += 2.0 * M_PI;
  return result;
}

} // namespace consai_vision_tracker
