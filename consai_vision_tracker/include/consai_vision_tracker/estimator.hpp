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

#ifndef CONSAI_VISION_TRACKER__ESTIMATOR_HPP_
#define CONSAI_VISION_TRACKER__ESTIMATOR_HPP_

#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/analyticconditionalgaussian.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>
#include <vector>

#include "robocup_ssl_msgs/msg/detection_ball.hpp"
#include "robocup_ssl_msgs/msg/detection_robot.hpp"
#include "robocup_ssl_msgs/msg/tracked_ball.hpp"
#include "robocup_ssl_msgs/msg/tracked_robot.hpp"

namespace consai_vision_tracker
{

using namespace robocup_ssl_msgs::msg;

class Estimator
{
public:
  Estimator();
  Estimator(const int team_color, const int id);

  void push_back_observation(const DetectionBall & ball);
  void push_back_observation(const DetectionRobot & robot);
  TrackedBall update_ball(const double dt = 0.0166);
  TrackedRobot update_robot(const double dt = 0.0166);

private:
  bool is_outlier(const TrackedBall & observation);
  bool is_outlier(const TrackedRobot & observation);

  std::vector<TrackedBall> ball_observations_;
  std::vector<TrackedRobot> robot_observations_;
  TrackedBall prev_tracked_ball_;
  TrackedRobot prev_tracked_robot_;
};

}  // namespace consai_vision_tracker

#endif  // CONSAI_VISION_TRACKER__ESTIMATOR_HPP_
