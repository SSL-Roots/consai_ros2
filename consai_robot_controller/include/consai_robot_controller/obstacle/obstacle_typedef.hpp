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

#ifndef CONSAI_ROBOT_CONTROLLER__OBSTACLE_TYPEDEF_HPP_
#define CONSAI_ROBOT_CONTROLLER__OBSTACLE_TYPEDEF_HPP_

namespace obstacle
{

class Point
{
public:
  Point(const double x, const double y)
  : x_(x), y_(y)
  {
  }

  double x() const
  {
    return x_;
  }

  double y() const
  {
    return y_;
  }

private:
  double x_;
  double y_;
};

using Radius = double;
using Position = Point;

}  // namespace obstacle

#endif  // CONSAI_ROBOT_CONTROLLER__OBSTACLE_TYPEDEF_HPP_
