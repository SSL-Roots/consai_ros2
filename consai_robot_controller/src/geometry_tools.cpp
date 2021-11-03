// Copyright 2021 Roots
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


#include <cmath>
#include "consai_robot_controller/geometry_tools.hpp"

namespace geometry_tools
{

double calc_angle(const State & from_pose, const State & to_pose)
{
  // from_poseからto_poseへの角度を計算する
  double diff_x = to_pose.x - from_pose.x;
  double diff_y = to_pose.y - from_pose.y;

  return std::atan2(diff_y, diff_x);
}

double normalize_theta(const double theta)
{
  // 角度を-pi ~ piに納める
  double retval = theta;
  while(retval >= M_PI) retval -= 2.0 * M_PI;
  while(retval <= -M_PI) retval += 2.0 * M_PI;
  return retval;
}

double distance(const State & pose1, const State & pose2)
{
  double diff_x = pose1.x - pose2.x;
  double diff_y = pose1.y - pose2.y;

  return std::hypot(diff_x, diff_y);
}

State pose_state(const TrackedRobot & robot) {
  State state;
  state.x = robot.pos.x;
  state.y = robot.pos.y;
  state.theta = robot.orientation;
  return state;
}

State pose_state(const TrackedBall & ball) {
  State state;
  state.x = ball.pos.x;
  state.y = ball.pos.y;
  state.theta = 0.0;
  return state;
}

double to_radians(const double degrees) {
  constexpr double TO_RADIANS = M_PI / 180.0;
  return degrees * TO_RADIANS;
}

double to_degrees(const double radians) {
  constexpr double TO_DEGREES = 180.0 / M_PI;
  return radians * TO_DEGREES;
}

Trans::Trans(const State & center, const double theta)
{
  center_ = std::complex<double>(center.x, center.y);
  theta_ = normalize_theta(theta);
  rotation_ = std::polar(1.0, theta_);
}

State Trans::transform(const State & pose) const 
{
  std::complex<double> point(pose.x, pose.y);
  auto transformed_point = (point - center_) * std::conj(rotation_);

  State transformed_pose;
  transformed_pose.x = transformed_point.real();
  transformed_pose.y = transformed_point.imag();
  transformed_pose.theta = transform_theta(pose.theta);
  return transformed_pose;
}

State Trans::inverted_transform(const State & pose) const 
{
  std::complex<double> point(pose.x, pose.y);
  auto inverted_point = point * rotation_ + center_;

  State inverted_pose;
  inverted_pose.x = inverted_point.real();
  inverted_pose.y = inverted_point.imag();
  inverted_pose.theta = inverted_transform_theta(pose.theta);
  return inverted_pose;
}

State Trans::inverted_transform(const double x, const double y, const double theta) const {
  std::complex<double> point(x, y);
  auto inverted_point = point * rotation_ + center_;

  State inverted_pose;
  inverted_pose.x = inverted_point.real();
  inverted_pose.y = inverted_point.imag();
  inverted_pose.theta = inverted_transform_theta(theta);
  return inverted_pose;
}

double Trans::transform_theta(const double theta) const 
{
  return normalize_theta(theta - theta_);
}

double Trans::inverted_transform_theta(const double theta) const 
{
  return normalize_theta(theta + theta_);
}

}