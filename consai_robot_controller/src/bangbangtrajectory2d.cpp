// Copyright 2023 Roots
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
#include <vector>

#include "consai_robot_controller/trajectory/trajectory.hpp"
#include "consai_robot_controller/trajectory/bangbangtrajectory1d.hpp"
#include "consai_robot_controller/trajectory/bangbangtrajectory2d.hpp"

BangBangTrajectory2D::BangBangTrajectory2D()
{
  _x = BangBangTrajectory1D();
  _y = BangBangTrajectory1D();
}

BangBangTrajectory2D::~BangBangTrajectory2D()
{
}

Vector2D BangBangTrajectory2D::get_position_mm(double t)
{
  return Vector2D(_x.get_position_mm(t), _y.get_position_mm(t));
}

Vector2D BangBangTrajectory2D::get_position(double t)
{
  return Vector2D(_x.get_position(t), _y.get_position(t));
}

Vector2D BangBangTrajectory2D::get_velocity(double t)
{
  return Vector2D(_x.get_velocity(t), _y.get_velocity(t));
}

Vector2D BangBangTrajectory2D::get_acceleration(double t)
{
  return Vector2D(_x.get_acceleration(t), _y.get_acceleration(t));
}

double BangBangTrajectory2D::get_total_time()
{
  return std::max(_x.get_total_time(), _y.get_total_time());
}

std::vector<double> BangBangTrajectory2D::get_time_sections()
{
  std::vector<double> sections = _x.get_time_sections();
  std::vector<double> y_sections = _y.get_time_sections();
  sections.insert(sections.end(), y_sections.begin(), y_sections.end());
  return sections;
}

Vector2D BangBangTrajectory2D::get_max_position()
{
  return Vector2D(_x.get_max_position(), _y.get_max_position());
}

Vector2D BangBangTrajectory2D::get_min_position()
{
  return Vector2D(_x.get_min_position(), _y.get_min_position());
}

void BangBangTrajectory2D::generate(
  Vector2D s0, Vector2D s1, Vector2D v0, double vmax, double acc, double accuracy
)
{
  double s0x = s0.x;
  double s0y = s0.y;
  double s1x = s1.x;
  double s1y = s1.y;
  double v0x = v0.x;
  double v0y = v0.y;

  double inc = M_PI / 8.0;
  double alpha = M_PI / 4.0;


  while (inc > 1e-7) {
    double cos = std::cos(alpha);
    double sin = std::sin(alpha);

    _x.generate(s0x, s1x, v0x, vmax * cos, acc * cos);
    _y.generate(s0y, s1y, v0y, vmax * sin, acc * sin);

    double diff = std::abs(_x.get_total_time() - _y.get_total_time());
    if (diff < accuracy) {
      break;
    }
    if (_x.get_total_time() > _y.get_total_time()) {
      alpha -= inc;
    } else {
      alpha += inc;
    }

    inc *= 0.5;
  }
}
