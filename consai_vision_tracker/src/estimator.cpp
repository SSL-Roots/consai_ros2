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
  prev_tracked_ball_.visibility.push_back(1.0);
}

Estimator::Estimator(const int team_color, const int id)
{
  prev_tracked_robot_.robot_id.team_color = team_color;
  prev_tracked_robot_.robot_id.id = id;
  prev_tracked_robot_.visibility.push_back(1.0);
}

void Estimator::push_back_observation(const DetectionBall & ball)
{
  TrackedBall observation;
  observation.pos.x = ball.x * 0.001;  // mm to meters
  observation.pos.y = ball.y * 0.001;  // mm to meters
  ball_observations_.push_back(observation);
}

void Estimator::push_back_observation(const DetectionRobot & robot)
{
  // 角度データを含まない場合は、そのデータを参照しない
  if(robot.orientation.size() == 0){
    return;
  }

  TrackedRobot observation;
  observation.pos.x = robot.x * 0.001;  // mm to meters
  observation.pos.y = robot.y * 0.001;  // mm to meters
  observation.orientation = robot.orientation[0];
  robot_observations_.push_back(observation);
}

TrackedBall Estimator::update_ball(const double dt)
{
  // 観測値から外れ値を取り除く
  for (auto it = ball_observations_.begin(); it != ball_observations_.end();){
    if(is_outlier(*it)){
      it = ball_observations_.erase(it);
    }else{
      ++it;
    }
  }

  auto size = ball_observations_.size();
  if(size == 0){
    // 観測値が無い場合の処理
    // visibilityを下げる
    prev_tracked_ball_.visibility[0] -= 0.002;
    if(prev_tracked_ball_.visibility[0] < 0){
      prev_tracked_ball_.visibility[0] = 0.0;
    }

  }else{
    // 観測値が複数ある場合は、その平均値をもとめる
    // この処理で観測値を全て削除する
    TrackedBall mean_observation;
    mean_observation.pos.x = 0.0;
    mean_observation.pos.y = 0.0;
    
    for (auto it = ball_observations_.begin(); it != ball_observations_.end();){
      mean_observation.pos.x += it->pos.x;
      mean_observation.pos.y += it->pos.y;
      it = ball_observations_.erase(it);
    }
    mean_observation.pos.x /= size;
    mean_observation.pos.y /= size;

    prev_tracked_ball_.pos.x = mean_observation.pos.x;
    prev_tracked_ball_.pos.y = mean_observation.pos.y;
    prev_tracked_ball_.visibility[0] = 1.0;
  }

  return prev_tracked_ball_;
}

TrackedRobot Estimator::update_robot(const double dt)
{
  // 観測値から外れ値を取り除く
  for (auto it = robot_observations_.begin(); it != robot_observations_.end();){
    if(is_outlier(*it)){
      it = robot_observations_.erase(it);
    }else{
      ++it;
    }
  }

  auto size = robot_observations_.size();
  if(size == 0){
    // 観測値が無い場合の処理
    // visibilityを下げる
    prev_tracked_robot_.visibility[0] -= 0.002;
    if(prev_tracked_robot_.visibility[0] < 0){
      prev_tracked_robot_.visibility[0] = 0.0;
    }

  }else{
    // 観測値が複数ある場合は、その平均値をもとめる
    // この処理で観測値を全て削除する
    TrackedRobot mean_observation;
    mean_observation.pos.x = 0.0;
    mean_observation.pos.y = 0.0;
    double sum_x = 0.0;
    double sum_y = 0.0;
    
    for (auto it = robot_observations_.begin(); it != robot_observations_.end();){
      mean_observation.pos.x += it->pos.x;
      mean_observation.pos.y += it->pos.y;
      sum_x += std::cos(it->orientation);  // 角度は-pi ~ piの範囲なので、2次元ベクトルに変換してから平均値を求める
      sum_y += std::sin(it->orientation);
      it = robot_observations_.erase(it);
    }
    mean_observation.pos.x /= size;
    mean_observation.pos.y /= size;
    sum_x /= size;
    sum_y /= size;
    mean_observation.orientation = std::fmod(std::atan2(sum_y, sum_x), M_PI);

    prev_tracked_robot_.pos.x = mean_observation.pos.x;
    prev_tracked_robot_.pos.y = mean_observation.pos.y;
    prev_tracked_robot_.orientation = mean_observation.orientation;
    prev_tracked_robot_.visibility[0] = 1.0;
  }

  return prev_tracked_robot_;
}

bool Estimator::is_outlier(const TrackedBall & observation)
{
  // TODO:implement this.
  return false;
}

bool Estimator::is_outlier(const TrackedRobot & observation)
{
  // TODO:implement this.
  return false;
}

}  // namespace consai_vision_tracker
