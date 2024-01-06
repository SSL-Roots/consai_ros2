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

#pragma once

#include <iostream>
#include <cmath>

class Field
{
public:
  Field(
    double x_size, double y_size, double goal_area_x_size, double goal_area_y_size,
    double center_circle_radius)
  {
    this->x_size = x_size;
    this->y_size = y_size;

    x_max = x_size / 2;
    x_min = -x_size / 2;
    y_max = y_size / 2;
    y_min = -y_size / 2;

    // ゴールエリア
    this->goal_area_x_size = goal_area_x_size;
    this->goal_area_y_size = goal_area_y_size;

    goal_area_0_left = x_min;
    goal_area_0_right = goal_area_0_left + goal_area_x_size;
    goal_area_0_top = goal_area_y_size / 2;
    goal_area_0_bottom = -goal_area_y_size / 2;

    goal_area_1_right = x_max;
    goal_area_1_left = goal_area_1_right - goal_area_x_size;
    goal_area_1_top = goal_area_y_size / 2;
    goal_area_1_bottom = -goal_area_y_size / 2;

    // センターサークル
    this->center_circle_radius = center_circle_radius;
  }

private:
  double x_size;
  double y_size;
  double x_max;
  double x_min;
  double y_max;
  double y_min;

  double goal_area_x_size;
  double goal_area_y_size;
  double goal_area_0_left;
  double goal_area_0_right;
  double goal_area_0_top;
  double goal_area_0_bottom;
  double goal_area_1_left;
  double goal_area_1_right;
  double goal_area_1_top;
  double goal_area_1_bottom;

  double center_circle_radius;
};
