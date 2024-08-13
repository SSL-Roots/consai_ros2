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

#include "consai_vision_tracker/ball_kalman_filter.hpp"
#include "robocup_ssl_msgs/msg/vector3.hpp"

#include <iostream>

namespace consai_vision_tracker
{

using SSLVector3 = robocup_ssl_msgs::msg::Vector3;

BallKalmanFilter::BallKalmanFilter(
  const double dt, const double lifetime,
  const double visibility_increase_rate,
  const double outlier_time_limit) :
  DT_(dt),
  VISIBILITY_CONTROL_VALUE_(dt / lifetime),
  VISIBILITY_INCREASE_RATE_(visibility_increase_rate),
  OUTLIER_COUNT_THRESHOLD_(outlier_time_limit / dt)
{

  // State transition matrix
  F_ = Matrix4d::Identity();
  F_(0, 2) = DT_;
  F_(1, 3) = DT_;

  // Observation matrix
  H_ = Matrix24d::Zero();
  H_(0, 0) = 1.0;
  H_(1, 1) = 1.0;

  // Process noise covariance matrix
  // Observation noise covariance matrix
  update_noise_covariance_matrix(0.1, 0.5, 0.03);

  // State vector
  // x_ = (x, y, vx, vy)
  // Error covariance matrix
  reset_x_and_p(Vector2d::Zero());
}

void BallKalmanFilter::push_back_observation(const DetectionBall & ball)
{
  TrackedBall observation;
  observation.pos.x = ball.x * 0.001;  // mm to meters
  observation.pos.y = ball.y * 0.001;  // mm to meters
  ball_observations_.push_back(observation);
}

TrackedBall BallKalmanFilter::update(const bool use_uncertain_sys_model)
{
  // Select process noise covariance matrix
  Matrix4d Q = Q_;
  if (use_uncertain_sys_model) {
    Q = Q_uncertain_;
  }

  Vector4d x_pred;
  Matrix4d P_pred;

  auto prediction = [&](void) {
    x_pred = F_ * x_;
    P_pred = F_ * P_ * F_.transpose() + Q;
  };

  auto calc_chi_square = [&](const Vector2d & z) {
    Matrix2d S = H_ * P_pred * H_.transpose() + R_;
    Vector2d innovation = z - H_ * x_pred;
    double chi_square = innovation.transpose() * S.inverse() * innovation;
    return chi_square;
  };

  // Prediction step
  prediction();

  // Remove outliers
  for (auto it = ball_observations_.begin(); it != ball_observations_.end(); ) {
    const Vector2d z(it->pos.x, it->pos.y);
    const double chi_square = calc_chi_square(z);

    if (!is_outlier(chi_square)) {
      outlier_count_ = 0;
      it++;
      continue;
    }

    outlier_count_++;
    if (outlier_count_ < OUTLIER_COUNT_THRESHOLD_) {
      it = ball_observations_.erase(it);
    } else {
      reset_x_and_p(z);
      prediction();  // Use new state vector
      it++;
    }
  }

  // Make observation
  Vector2d z(x_[0], x_[1]);
  if (ball_observations_.empty()) {
    visibility_ = std::max(0.0, visibility_ - VISIBILITY_CONTROL_VALUE_);
  } else {
    z = make_observation();
    visibility_ = std::min(1.0, visibility_ + VISIBILITY_INCREASE_RATE_ * VISIBILITY_CONTROL_VALUE_);
    ball_observations_.clear();
  }

  // Calculate innovation
  Vector2d innovation = z - H_ * x_pred;
  // Calculate innovation covariance matrix
  Matrix2d S = H_ * P_pred * H_.transpose() + R_;

  // Update step
  Matrix42d K = P_pred * H_.transpose() * S.inverse();
  x_ = x_pred + K * innovation;
  P_ = (Matrix4d::Identity() - K * H_) * P_pred;

  return get_estimation();
}

TrackedBall BallKalmanFilter::get_estimation(void) const
{
  TrackedBall estimation;
  estimation.pos.x = x_[0];
  estimation.pos.y = x_[1];

  SSLVector3 vel;
  vel.x = x_[2];
  vel.y = x_[3];
  estimation.vel.push_back(vel);
  estimation.visibility.push_back(visibility_);

  return estimation;
}

void BallKalmanFilter::update_noise_covariance_matrix(
    const double q_max_acc, const double q_uncertain_max_acc, const double r_pos_stddev)
{
  // Process noise covariance matrix
  auto gen_Q = [this](const double max_acc) {
    // Process noise depends on the maximum acceleration
    const double sigma_vel = max_acc * DT_;
    const double sigma_pos = 0.5 * max_acc * DT_ * DT_;
    const double var_vel = sigma_vel * sigma_vel;
    const double var_pos = sigma_pos * sigma_pos;
    const double var_vel_pos = sigma_vel * sigma_pos;

    Matrix4d Q;
    Q << var_pos, 0.0, var_vel_pos, 0.0,
         0.0, var_pos, 0.0, var_vel_pos,
         var_vel_pos, 0.0, var_vel, 0.0,
         0.0, var_vel_pos, 0.0, var_vel;
    return Q;
  };
  Q_ = gen_Q(q_max_acc);  // m/s^2
  Q_uncertain_ = gen_Q(q_uncertain_max_acc);  // m/s^2

  // Observation noise covariance matrix
  auto gen_R = [&](const double std_dev) {
    const double var = std_dev * std_dev;
    return Eigen::DiagonalMatrix<double, 2>(var, var);
  };
  R_ = gen_R(r_pos_stddev);  // meters
}

Vector2d BallKalmanFilter::make_observation(void) const
{
  if (ball_observations_.empty()) {
    return Vector2d(x_[0], x_[1]);
  }

  Vector2d mean_observation(0.0, 0.0);
  for (const auto & observation : ball_observations_) {
    mean_observation[0] += observation.pos.x;
    mean_observation[1] += observation.pos.y;
  }
  mean_observation /= ball_observations_.size();
  return mean_observation;
}

bool BallKalmanFilter::is_outlier(const double chi_squared_value) const
{
  constexpr double THRESHOLD = 5.991;  // 2 degrees of freedom, 0.05 significance level

  if (chi_squared_value > THRESHOLD) {
    return true;
  }
  return false;
}

void BallKalmanFilter::reset_x_and_p(const Vector2d & observation)
{
  x_ << observation[0], observation[1], 0.0, 0.0;
  P_ = Matrix4d::Identity() * 10.0;
}


} // namespace consai_vision_tracker
