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

#ifndef CONSAI_ROBOT_CONTROLLER__OBSTACLE_BALL_HPP_
#define CONSAI_ROBOT_CONTROLLER__OBSTACLE_BALL_HPP_

#include "consai_robot_controller/obstacle/obstacle_typedef.hpp"

namespace obstacle
{

class ObstacleBall
{
public:
  ObstacleBall(const Position & position, const Radius radius)
  : position_(position), radius_(radius)
  {
  }

  const Point & position() const
  {
    return position_;
  }

  Radius radius() const
  {
    return radius_;
  }

private:
  Position position_;
  Radius radius_;
};

}  // namespace obstacle

#endif  // CONSAI_ROBOT_CONTROLLER__OBSTACLE_BALL_HPP_
