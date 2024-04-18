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

#ifndef CONSAI_ROBOT_CONTROLLER__GLOBAL_FOR_DEBUG_HPP_
#define CONSAI_ROBOT_CONTROLLER__GLOBAL_FOR_DEBUG_HPP_


#include "consai_robot_controller/trajectory/utils.hpp"

namespace global_for_debug
{
extern Velocity2D last_control_output_ff;
extern Velocity2D last_control_output_p;
}  // namespace global_for_debug

#endif  // CONSAI_ROBOT_CONTROLLER__GLOBAL_FOR_DEBUG_HPP_
