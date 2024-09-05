// Copyright 2022 Roots
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

#include <gtest/gtest.h>
#include "consai_robot_controller/tools/control_tools.hpp"

// tanh関数を用いた速度制御のテスト
TEST(TestControlTools, velocity_contol_tanh) {
  // 誤差の範囲
  const double ABS_ERROR = 0.05;
  // 最大速度
  const double MAX_VEL = 5.0;
  // tanh関数の正から負へ変化する曲線部の区間の調整
  double range = 1.0;
  EXPECT_NEAR(control_tools::velocity_contol_tanh(0.0, range, MAX_VEL), 0.0 * MAX_VEL, ABS_ERROR);
  EXPECT_NEAR(control_tools::velocity_contol_tanh(1.0, range, MAX_VEL), 0.76 * MAX_VEL, ABS_ERROR);
  EXPECT_NEAR(control_tools::velocity_contol_tanh(2.0, range, MAX_VEL), 0.96 * MAX_VEL, ABS_ERROR);
  EXPECT_NEAR(control_tools::velocity_contol_tanh(5.0, range, MAX_VEL), 0.99 * MAX_VEL, ABS_ERROR);
  EXPECT_NEAR(
    control_tools::velocity_contol_tanh(-1.0, range, MAX_VEL), -0.76 * MAX_VEL,
    ABS_ERROR);
  EXPECT_NEAR(
    control_tools::velocity_contol_tanh(-2.0, range, MAX_VEL), -0.96 * MAX_VEL,
    ABS_ERROR);
  EXPECT_NEAR(
    control_tools::velocity_contol_tanh(-5.0, range, MAX_VEL), -0.99 * MAX_VEL,
    ABS_ERROR);

  range = 2.0;
  EXPECT_NEAR(control_tools::velocity_contol_tanh(0.5, range, MAX_VEL), 0.76 * MAX_VEL, ABS_ERROR);
  EXPECT_NEAR(control_tools::velocity_contol_tanh(1.0, range, MAX_VEL), 0.96 * MAX_VEL, ABS_ERROR);
  EXPECT_NEAR(control_tools::velocity_contol_tanh(2.5, range, MAX_VEL), 0.99 * MAX_VEL, ABS_ERROR);
  EXPECT_NEAR(
    control_tools::velocity_contol_tanh(-0.5, range, MAX_VEL), -0.76 * MAX_VEL,
    ABS_ERROR);
  EXPECT_NEAR(
    control_tools::velocity_contol_tanh(-1.0, range, MAX_VEL), -0.96 * MAX_VEL,
    ABS_ERROR);
  EXPECT_NEAR(
    control_tools::velocity_contol_tanh(-2.5, range, MAX_VEL), -0.99 * MAX_VEL,
    ABS_ERROR);
}

// sin関数を用いた角速度制御のテスト
TEST(TestControlTools, angular_velocity_contol_sin) {
  // 誤差の範囲
  const double ABS_ERROR = 0.05;
  // 最大速度
  const double MAX_VEL = 5.0;

  EXPECT_NEAR(control_tools::angular_velocity_contol_sin(M_PI, MAX_VEL), MAX_VEL, ABS_ERROR);
  EXPECT_NEAR(control_tools::angular_velocity_contol_sin(M_PI_2, MAX_VEL), MAX_VEL, ABS_ERROR);
  EXPECT_NEAR(
    control_tools::angular_velocity_contol_sin(M_PI_4, MAX_VEL), 0.71 * MAX_VEL,
    ABS_ERROR);
  EXPECT_NEAR(
    control_tools::angular_velocity_contol_sin(
      M_PI_4 / 2.0,
      MAX_VEL), 0.38 * MAX_VEL, ABS_ERROR);
  EXPECT_NEAR(control_tools::angular_velocity_contol_sin(0, MAX_VEL), 0.0 * MAX_VEL, ABS_ERROR);
  EXPECT_NEAR(
    control_tools::angular_velocity_contol_sin(
      -M_PI_4 / 2.0,
      MAX_VEL), -0.38 * MAX_VEL, ABS_ERROR);
  EXPECT_NEAR(
    control_tools::angular_velocity_contol_sin(
      -M_PI_4,
      MAX_VEL), -0.71 * MAX_VEL, ABS_ERROR);
  EXPECT_NEAR(control_tools::angular_velocity_contol_sin(-M_PI_2, MAX_VEL), -MAX_VEL, ABS_ERROR);
  EXPECT_NEAR(control_tools::angular_velocity_contol_sin(-M_PI, MAX_VEL), -MAX_VEL, ABS_ERROR);
}
