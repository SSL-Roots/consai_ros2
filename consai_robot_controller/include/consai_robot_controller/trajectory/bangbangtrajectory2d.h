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

// bangbangtrajectory2d.h
#ifndef BANGBANGTRAJECTORY2D_H
#define BANGBANGTRAJECTORY2D_H

#include "consai_robot_controller/trajectory/trajectory.h"
#include "consai_robot_controller/trajectory/bangbangtrajectory1d.h"

class BangBangTrajectory2D
{
public:
  BangBangTrajectory2D();
  ~BangBangTrajectory2D();

  Vector2D get_position_mm(double t);
  Vector2D get_position(double t);
  Vector2D get_velocity(double t);
  Vector2D get_acceleration(double t);
  double get_total_time();
  PosVelAcc get_values_at_time(double t);
  std::vector < double > get_time_sections();
  Vector2D get_max_position();
  Vector2D get_min_position();

  void generate(
    Vector2D s0, Vector2D s1, Vector2D v0, double vmax, double acc, double accuracy
  );

private:
  BangBangTrajectory1D _x;
  BangBangTrajectory1D _y;

};

#endif // BANGBANGTRAJECTORY2D_H
