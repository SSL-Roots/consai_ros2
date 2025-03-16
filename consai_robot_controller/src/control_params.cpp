// Copyright 2025 Roots
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

#include "consai_robot_controller/control_params.hpp"

namespace consai_robot_controller
{

ControlParams::ControlParams()
: hard_limit_acceleration_xy(2.0),
  soft_limit_acceleration_xy(2.0 * 0.8),
  hard_limit_acceleration_theta(2.0 * M_PI),
  soft_limit_acceleration_theta(2.0 * M_PI * 0.8),
  hard_limit_velocity_xy(2.0),
  soft_limit_velocity_xy(2.0 * 0.8),
  hard_limit_velocity_theta(2.0 * M_PI),
  soft_limit_velocity_theta(2.0 * M_PI * 0.8),
  control_a_theta(0.5),
  p_gain_xy(1.5),
  d_gain_xy(0.0),
  p_gain_theta(2.5),
  d_gain_theta(0.0)
{
}

}  // namespace consai_robot_controller