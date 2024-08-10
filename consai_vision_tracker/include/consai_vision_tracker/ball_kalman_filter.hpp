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

#ifndef CONSAI_VISION_TRACKER__BALL_KALMAN_FILTER_HPP_
#define CONSAI_VISION_TRACKER__BALL_KALMAN_FILTER_HPP_

#include <Eigen/Dense>
#include <vector>

#include "robocup_ssl_msgs/msg/detection_ball.hpp"
#include "robocup_ssl_msgs/msg/tracked_ball.hpp"


namespace consai_vision_tracker
{

using DetectionBall = robocup_ssl_msgs::msg::DetectionBall;
using TrackedBall = robocup_ssl_msgs::msg::TrackedBall;
using Vector4d = Eigen::Vector4d;
using Vector2d = Eigen::Vector2d;
using Matrix4d = Eigen::Matrix4d;
using Matrix2d = Eigen::Matrix2d;
using Matrix24d = Eigen::Matrix<double, 2, 4>;
using Matrix42d = Eigen::Matrix<double, 4, 2>;

class BallKalmanFilter
{
 public:
  explicit BallKalmanFilter(const double dt = 0.01);

  void push_back_observation(const DetectionBall & ball);
  TrackedBall update(const bool use_uncertain_sys_model);
  TrackedBall get_estimation(void) const;

 private:
  std::vector<TrackedBall> ball_observations_;
  TrackedBall prev_tracked_ball_;

  Vector4d x_;

  Matrix4d F_;
  Matrix24d H_;
  Matrix4d Q_;
  Matrix2d R_;
  Matrix4d P_;
};

} // namespace consai_vision_tracker

#endif  // CONSAI_VISION_TRACKER__BALL_KALMAN_FILTER_HPP_
