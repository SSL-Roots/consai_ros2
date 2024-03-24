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

#ifndef CONSAI_ROBOT_CONTROLLER__OBSTACLE_OBSERVER_HPP_
#define CONSAI_ROBOT_CONTROLLER__OBSTACLE_OBSERVER_HPP_

#include <memory>

#include "consai_msgs/action/robot_control.hpp"
#include "consai_robot_controller/detection_extractor.hpp"
#include "consai_robot_controller/obstacle/obstacle_environment.hpp"
#include "robocup_ssl_msgs/msg/tracked_robot.hpp"

namespace obstacle
{

using RobotControl = consai_msgs::action::RobotControl;
using TrackedRobot = robocup_ssl_msgs::msg::TrackedRobot;

class ObstacleObserver
{
public:
  ObstacleObserver(
    const std::shared_ptr<parser::DetectionExtractor> & detection_extractor);

  ObstacleEnvironment get_obstacle_environment(
    const std::shared_ptr<const RobotControl::Goal> goal,
    const TrackedRobot & my_robot) const;

private:
  std::shared_ptr<parser::DetectionExtractor> detection_;
};

}  // namespace obstacle

#endif  // CONSAI_ROBOT_CONTROLLER__OBSTACLE_OBSERVER_HPP_
