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

#ifndef CONSAI_ROBOT_CONTROLLER__PROHIBITED_AREA_HPP_
#define CONSAI_ROBOT_CONTROLLER__PROHIBITED_AREA_HPP_

namespace obstacle
{

class ProhibitedArea
{
public:
  ProhibitedArea(const double p1_x, const double p1_y, const double p2_x, const double p2_y)
  : p1_x_(p1_x), p1_y_(p1_y), p2_x_(p2_x), p2_y_(p2_y)
  {
  }

  double get_p1_x() const
  {
    return p1_x_;
  }

  double get_p1_y() const
  {
    return p1_y_;
  }

  double get_p2_x() const
  {
    return p2_x_;
  }

  double get_p2_y() const
  {
    return p2_y_;
  }

private:
  double p1_x_;
  double p1_y_;
  double p2_x_;
  double p2_y_;
};

}  // namespace obstacle

#endif  // CONSAI_ROBOT_CONTROLLER__PROHIBITED_AREA_HPP_
