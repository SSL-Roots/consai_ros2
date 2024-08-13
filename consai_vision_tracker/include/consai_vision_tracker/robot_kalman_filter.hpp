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

#ifndef CONSAI_VISION_TRACKER__ROBOT_KALMAN_FILTER_HPP_
#define CONSAI_VISION_TRACKER__ROBOT_KALMAN_FILTER_HPP_

#include <Eigen/Dense>
#include <vector>

#include "consai_msgs/msg/robot_local_velocity.hpp"
#include "robocup_ssl_msgs/msg/detection_robot.hpp"
#include "robocup_ssl_msgs/msg/robot_id.hpp"
#include "robocup_ssl_msgs/msg/tracked_robot.hpp"


namespace consai_vision_tracker
{

using DetectionRobot = robocup_ssl_msgs::msg::DetectionRobot;
using RobotId = robocup_ssl_msgs::msg::RobotId;
using TrackedRobot = robocup_ssl_msgs::msg::TrackedRobot;
using RobotLocalVelocity = consai_msgs::msg::RobotLocalVelocity;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix36d = Eigen::Matrix<double, 3, 6>;

class RobotKalmanFilter
{
public:
  explicit RobotKalmanFilter(
    const int team_color, const int id,
    const double dt = 0.01, const double lifetime = 2.0,
    const double visibility_increase_rate = 5.0,
    const double outlier_time_limit = 0.1);

  void push_back_observation(const DetectionRobot & robot);
  TrackedRobot update(void);
  TrackedRobot get_estimation(void) const;
  void update_noise_covariance_matrix(
    const double q_max_acc_xy, const double q_max_acc_theta,
    const double r_pos_stddev_xy, const double r_pos_stddev_theta);
  RobotLocalVelocity calc_local_velocity(void) const;

private:
  Vector3d make_observation(void) const;
  bool is_outlier(const double chi_squared_value) const;
  void reset_x_and_p(const Vector3d & observation);
  double normalize_angle(const double angle) const;

  const double DT_;
  const double VISIBILITY_CONTROL_VALUE_;
  const double VISIBILITY_INCREASE_RATE_;
  const int OUTLIER_COUNT_THRESHOLD_;
  const double OMEGA_ZERO_ = 1.0e-6;

  double visibility_ = 1.0;
  int outlier_count_ = 0;

  RobotId robot_id_;
  std::vector<TrackedRobot> robot_observations_;

  Vector6d x_;
  Matrix6d F_;
  Matrix36d H_;
  Matrix6d Q_;
  Matrix3d R_;
  Matrix6d P_;
};

} // namespace consai_vision_tracker

#endif  // CONSAI_VISION_TRACKER__ROBOT_KALMAN_FILTER_HPP_
