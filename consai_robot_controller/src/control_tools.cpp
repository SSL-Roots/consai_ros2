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

#include <algorithm>
#include <cmath>
#include <iostream>

#include "consai_robot_controller/control_tools.hpp"

namespace control_tools
{
  // double velocity_contol_tanh(const double diff, const double range, const double a, const double max_vel)
  double velocity_contol_tanh(double diff, double range, double a, double max_vel)
  {
    // tanh関数を用いた速度の設定
    double vel = std::tanh(range * diff) * a * max_vel;
    return vel;
  }

  double angular_velocity_contol_sin(const double diff, const double a, const double max_vel)
  {
    // sin関数を用いた角速度の設定
    double vel = a * max_vel;
    // -pi/2 ~ pi/2の間であればsin関数により速度を設定
    if (-M_PI_2 < diff && diff < M_PI_2) {
      vel = std::sin(diff) * a * max_vel;
    }
    return vel;
  }
}
