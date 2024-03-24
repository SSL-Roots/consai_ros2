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

#ifndef CONSAI_ROBOT_CONTROLLER__TOOLS__GEOMETRY_TOOLS_HPP_
#define CONSAI_ROBOT_CONTROLLER__TOOLS__GEOMETRY_TOOLS_HPP_

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
double to_radians(const double degrees);
double to_degrees(const double radians);
State intersection(const State & p1, const State & p2, const State & p3, const State & p4);

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

#endif  // CONSAI_ROBOT_CONTROLLER__TOOLS__GEOMETRY_TOOLS_HPP_
