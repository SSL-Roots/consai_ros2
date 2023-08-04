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

#include "obstacle_typedef.hpp"

namespace obstacle
{

class ProhibitedArea
{
public:
  ProhibitedArea(const Position & p1, const Position & p2)
  : p1_(p1), p2_(p2)
  {
  }

  const Position & p1() const
  {
    return p1_;
  }

  const Position & p2() const
  {
    return p2_;
  }

private:
  Position p1_;
  Position p2_;
};

}  // namespace obstacle

#endif  // CONSAI_ROBOT_CONTROLLER__PROHIBITED_AREA_HPP_
