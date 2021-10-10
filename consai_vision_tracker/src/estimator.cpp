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

#include <iostream>

#include "consai_vision_tracker/estimator.hpp"

namespace consai_vision_tracker
{

Estimator::Estimator()
{
  observation_ball_.visibility.push_back(1.0);
}

Estimator::Estimator(const int team_color, const int id)
{
  team_color_ = team_color;
  id_ = id;
  observation_robot_.visibility.push_back(1.0);
}

void Estimator::set_observation(const DetectionBall & ball, const int camera_id, const double t_capture)
{
  observation_ball_.pos.x = ball.x * 0.001;  // mm to meters
  observation_ball_.pos.y = ball.y * 0.001;  // mm to meters
  observation_ball_.visibility[0] = 1.0;
}

void Estimator::set_observation(const DetectionRobot & robot, const int camera_id, const double t_capture)
{
  // 角度データを含まない場合は、そのデータを参照しない
  if(robot.orientation.size() == 0){
    return;
  }

  observation_robot_.pos.x = robot.x * 0.001;  // mm to meters
  observation_robot_.pos.y = robot.y * 0.001;  // mm to meters
  observation_robot_.orientation = robot.orientation[0];
  observation_robot_.visibility[0] = 1.0;
}

TrackedBall Estimator::estimate_ball(const double dt)
{
  TrackedBall tracked_ball;
  tracked_ball.pos.x = observation_ball_.pos.x;
  tracked_ball.pos.y = observation_ball_.pos.y;
  tracked_ball.visibility = observation_ball_.visibility;

  // estimateするたびにvisibilityを下げる
  // obseravationがセットされるたびに、visibilityは1.0に戻る
  observation_ball_.visibility[0] -= 0.002;
  if(observation_ball_.visibility[0] < 0){
    observation_ball_.visibility[0] = 0.0;
  }

  return tracked_ball;
}

TrackedRobot Estimator::estimate_robot(const double dt)
{
  TrackedRobot tracked_robot;

  tracked_robot.pos.x = observation_robot_.pos.x;
  tracked_robot.pos.y = observation_robot_.pos.y;
  tracked_robot.orientation = observation_robot_.orientation;
  tracked_robot.visibility = observation_robot_.visibility;
  tracked_robot.robot_id.team_color = team_color_;
  tracked_robot.robot_id.id= id_;

  // estimateするたびにvisibilityを下げる
  // obseravationがセットされるたびに、visibilityは1.0に戻る
  observation_robot_.visibility[0] -= 0.002;
  if(observation_robot_.visibility[0] < 0){
    observation_robot_.visibility[0] = 0.0;
  }

  return tracked_robot;
}

}  // namespace consai_vision_tracker
