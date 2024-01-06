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

// bangbangtrajectory3d.h
#ifndef BANGBANGTRAJECTORY3D_H
#define BANGBANGTRAJECTORY3D_H

#include "consai_robot_controller/trajectory/trajectory.h"
#include "consai_robot_controller/trajectory/bangbangtrajectory1d.h"
#include "consai_robot_controller/trajectory/bangbangtrajectory2d.h"

class BangBangTrajectory3D
{
public:
  BangBangTrajectory3D();
  ~BangBangTrajectory3D();

  Pose2D get_pose(double t);
  Velocity2D get_velocity(double t);
  double get_total_time();

  void generate(
    Pose2D s0, Pose2D s1, Velocity2D v0, double vmax_linear, double vmax_angular, double acc_linear,
    double acc_angular, double accuracy
  );

private:
  BangBangTrajectory2D linear_;
  BangBangTrajectory1D angular_;

};

#endif // BANGBANGTRAJECTORY3D_H
