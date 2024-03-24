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

#ifndef CONSAI_ROBOT_CONTROLLER__OBSTACLE_ENVIRONMENT_HPP_
#define CONSAI_ROBOT_CONTROLLER__OBSTACLE_ENVIRONMENT_HPP_

#include <vector>

#include "consai_robot_controller/obstacle/obstacle_robot.hpp"
#include "consai_robot_controller/obstacle/obstacle_ball.hpp"
#include "consai_robot_controller/obstacle/prohibited_area.hpp"

namespace obstacle
{

class ObstacleEnvironment
{
public:
  ObstacleEnvironment() = default;
  ~ObstacleEnvironment() = default;

  void append_obstacle_robot(const ObstacleRobot & obstacle_robot)
  {
    obstacle_robots_.push_back(obstacle_robot);
  }

  void append_obstacle_ball(const ObstacleBall & obstacle_ball)
  {
    obstacle_balls_.push_back(obstacle_ball);
  }

  void append_prohibited_area(const ProhibitedArea & prohibited_area)
  {
    prohibited_areas_.push_back(prohibited_area);
  }

  const std::vector<ObstacleRobot> & get_obstacle_robots() const
  {
    return obstacle_robots_;
  }

  const std::vector<ObstacleBall> & get_obstacle_balls() const
  {
    return obstacle_balls_;
  }

  const std::vector<ProhibitedArea> & get_prohibited_areas() const
  {
    return prohibited_areas_;
  }

  void clear_obstacle_robots()
  {
    obstacle_robots_.clear();
  }

  void clear_obstacle_balls()
  {
    obstacle_balls_.clear();
  }

  void clear_prohibited_areas()
  {
    prohibited_areas_.clear();
  }

  void clear_all()
  {
    clear_obstacle_robots();
    clear_obstacle_balls();
    clear_prohibited_areas();
  }

private:
  std::vector<ObstacleRobot> obstacle_robots_;
  std::vector<ObstacleBall> obstacle_balls_;
  std::vector<ProhibitedArea> prohibited_areas_;
};

}  // namespace obstacle

#endif  // CONSAI_ROBOT_CONTROLLER__OBSTACLE_ENVIRONMENT_HPP_
