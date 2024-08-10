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

namespace consai_vision_tracker
{

using SSLVector3 = robocup_ssl_msgs::msg::Vector3;

BallKalmanFilter::BallKalmanFilter(const double dt)
{
  // State vector
  // x_ = (x, y, vx, vy)
  x_ = Vector4d::Zero();

  // State transition matrix
  F_ = Matrix4d::Identity();
  F_(0, 2) = dt;
  F_(1, 3) = dt;

  // Observation matrix
  H_ = Matrix24d::Zero();
  H_(0, 0) = 1.0;
  H_(1, 1) = 1.0;

  // Process noise covariance matrix
  // Q_ = diag(qx, qy, qvx, qvy)
  Q_ = Matrix4d::Identity() * 0.01;

  // Observation noise covariance matrix
  // R_ = diag(rx, ry)
  R_ = Matrix2d::Identity() * 0.1;

  // Error covariance matrix
  P_ = Matrix4d::Identity();
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
  Vector2d z;

  // TODO(): Fix me
  if (ball_observations_.empty()) {
    z << x_[0], x_[1];
  } else {
    z[0] = ball_observations_.back().pos.x;
    z[1] = ball_observations_.back().pos.y;
    ball_observations_.clear();
  }

  // Prediction step
  Vector4d x_pred = F_ * x_;
  Matrix4d P_pred = F_ * P_ * F_.transpose() + Q_;

  // Calculate innovation
  Vector2d innovation = z - H_ * x_pred;

  // Calculate innovation covariance matrix
  Matrix2d S = H_ * P_pred * H_.transpose() + R_;

  // Calculate chi-square value
  double chi_square = innovation.transpose() * S.inverse() * innovation;

  // Check outlier
  // TODO(): Implement outlier detection

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
  estimation.visibility.push_back(1.0);

  return estimation;
}


} // namespace consai_vision_tracker
