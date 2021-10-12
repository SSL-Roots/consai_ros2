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

#include "consai_vision_tracker/ball_tracker.hpp"

namespace consai_vision_tracker
{

static const double VISIBILITY_CONTROL_VALUE = 0.002;

BallTracker::BallTracker()
{
  prev_tracked_ball_.visibility.push_back(1.0);
}

void BallTracker::push_back_observation(const DetectionBall & ball)
{
  TrackedBall observation;
  observation.pos.x = ball.x * 0.001;  // mm to meters
  observation.pos.y = ball.y * 0.001;  // mm to meters
  ball_observations_.push_back(observation);
}

TrackedBall BallTracker::update(const double dt)
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
    prev_tracked_ball_.visibility[0] -= VISIBILITY_CONTROL_VALUE;
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
    prev_tracked_ball_.visibility[0] += VISIBILITY_CONTROL_VALUE;
    if(prev_tracked_ball_.visibility[0] > 1.0){
      prev_tracked_ball_.visibility[0] = 1.0;
    }
  }

  return prev_tracked_ball_;
}

bool BallTracker::is_outlier(const TrackedBall & observation)
{
  // TODO:implement this.
  return false;
}

}  // namespace consai_vision_tracker
