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

}