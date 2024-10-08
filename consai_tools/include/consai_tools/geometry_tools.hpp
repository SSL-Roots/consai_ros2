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

#ifndef CONSAI_TOOLS__GEOMETRY_TOOLS_HPP_
#define CONSAI_TOOLS__GEOMETRY_TOOLS_HPP_

#include <complex>
#include <algorithm>

#include "consai_msgs/msg/state2_d.hpp"
#include "robocup_ssl_msgs/msg/tracked_ball.hpp"
#include "robocup_ssl_msgs/msg/tracked_robot.hpp"

namespace geometry_tools
{

using State = consai_msgs::msg::State2D;
using TrackedBall = robocup_ssl_msgs::msg::TrackedBall;
using TrackedRobot = robocup_ssl_msgs::msg::TrackedRobot;

double calc_angle(const State & from_pose, const State & to_pose);
double normalize_theta(const double theta);
double distance(const State & pose1, const State & pose2);
State pose_state(const TrackedRobot & robot);
State pose_state(const TrackedBall & ball);
State velocity_state(const TrackedRobot & robot);
State velocity_state(const TrackedBall & ball);
State gen_state(const double x, const double y, const double theta = 0.0);
double to_radians(const double degrees);
double to_degrees(const double radians);
State intersection(const State & p1, const State & p2, const State & p3, const State & p4);
bool is_same(
  const State & p1, const State & p2,
  const double distance_threshold = 0.01, const double theta_threshold = 0.17);
bool is_visible(const TrackedRobot & robot, const double threshold = 0.01);
bool is_visible(const TrackedBall & ball, const double threshold = 0.01);
bool is_lines_intersect(
  const State & line1_p1, const State & line1_p2, const State & line2_p1, const State & line2_p2);

class Trans
{
public:
  Trans(const State & center, const double theta);
  State transform(const State & pose) const;
  State inverted_transform(const State & pose) const;
  State inverted_transform(const double x, const double y, const double theta) const;
  double transform_theta(const double theta) const;
  double inverted_transform_theta(const double theta) const;

private:
  std::complex<double> center_;
  std::complex<double> rotation_;
  double theta_;
};

}  // namespace geometry_tools

#endif  // CONSAI_TOOLS__GEOMETRY_TOOLS_HPP_
