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


#ifndef CONSAI_ROBOT_CONTROLLER__CONTROL_PARAMS_HPP_
#define CONSAI_ROBOT_CONTROLLER__CONTROL_PARAMS_HPP_

#include <tuple>

namespace consai_robot_controller
{

class ControlParams
{
public:
  ControlParams();

  double hard_limit_acceleration_xy;
  double soft_limit_acceleration_xy;
  double hard_limit_acceleration_theta;
  double soft_limit_acceleration_theta;
  double hard_limit_velocity_xy;
  double soft_limit_velocity_xy;
  double hard_limit_velocity_theta;
  double soft_limit_velocity_theta;
  double control_a_theta;
  double p_gain_xy;
  double d_gain_xy;
  double p_gain_theta;
  double d_gain_theta;

private:
  auto tie() const
  {
    return std::tie(
      hard_limit_acceleration_xy, soft_limit_acceleration_xy,
      hard_limit_acceleration_theta, soft_limit_acceleration_theta,
      hard_limit_velocity_xy, soft_limit_velocity_xy,
      hard_limit_velocity_theta, soft_limit_velocity_theta,
      control_a_theta, p_gain_xy, d_gain_xy, p_gain_theta, d_gain_theta);
  }

public:
  bool operator==(const ControlParams & other) const
  {
    return tie() == other.tie();
  }

  bool operator!=(const ControlParams & other) const
  {
    return !(*this == other);
  }
};
}  // namespace consai_robot_controller

#endif  // CONSAI_ROBOT_CONTROLLER__CONTROL_PARAMS_HPP_
