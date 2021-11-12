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

#ifndef CONSAI_VISION_TRACKER__ROBOT_TRACKER_HPP_
#define CONSAI_VISION_TRACKER__ROBOT_TRACKER_HPP_

#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/analyticconditionalgaussian.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>
#include <memory>
#include <vector>

#include "robocup_ssl_msgs/msg/detection_robot.hpp"
#include "robocup_ssl_msgs/msg/tracked_robot.hpp"

namespace consai_vision_tracker
{

using DetectionRobot = robocup_ssl_msgs::msg::DetectionRobot;
using TrackedRobot = robocup_ssl_msgs::msg::TrackedRobot;
using ConditionalGaussian = BFL::LinearAnalyticConditionalGaussian;
using SystemModelGaussianUncertainty = BFL::LinearAnalyticSystemModelGaussianUncertainty;
using MeasurementModelGaussianUncertainty = BFL::LinearAnalyticMeasurementModelGaussianUncertainty;
using Gaussian = BFL::Gaussian;
using ExtendedKalmanFilter = BFL::ExtendedKalmanFilter;

class RobotTracker
{
public:
  RobotTracker(const int team_color, const int id, const double dt = 0.01);

  void push_back_observation(const DetectionRobot & robot);
  TrackedRobot update();

private:
  void reset_prior();
  bool is_outlier(const TrackedRobot & observation) const;
  void correct_orientation_overflow_of_prior();
  double normalize_orientation(double orientation) const;
  double normalize_orientation(const double from, const double to) const;

  std::vector<TrackedRobot> robot_observations_;
  TrackedRobot prev_tracked_robot_;

  std::shared_ptr<ConditionalGaussian> sys_pdf_;
  std::shared_ptr<SystemModelGaussianUncertainty> sys_model_;
  std::shared_ptr<ConditionalGaussian> meas_pdf_;
  std::shared_ptr<MeasurementModelGaussianUncertainty> meas_model_;
  std::shared_ptr<Gaussian> prior_;
  std::shared_ptr<ExtendedKalmanFilter> filter_;
};

}  // namespace consai_vision_tracker

#endif  // CONSAI_VISION_TRACKER__ROBOT_TRACKER_HPP_
