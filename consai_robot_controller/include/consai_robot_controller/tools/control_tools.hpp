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

#ifndef CONSAI_ROBOT_CONTROLLER__TOOLS__CONTROL_TOOLS_HPP_
#define CONSAI_ROBOT_CONTROLLER__TOOLS__CONTROL_TOOLS_HPP_

#include <complex>
#include <algorithm>

namespace control_tools
{

double velocity_contol_tanh(const double diff, const double range, const double max_vel);
double angular_velocity_contol_sin(const double diff, const double max_vel);

}  // namespace control_tools

#endif  // CONSAI_ROBOT_CONTROLLER__TOOLS__CONTROL_TOOLS_HPP_
