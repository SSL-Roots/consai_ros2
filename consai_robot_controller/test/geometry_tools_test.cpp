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
#include "consai_robot_controller/geometry_tools.hpp"

TEST(TestGeometryTools, to_radians) {
  const double ABS_ERROR = 0.00001;
  EXPECT_NEAR(geometry_tools::to_radians(0), 0, ABS_ERROR);
  EXPECT_NEAR(geometry_tools::to_radians(180), 3.141592, ABS_ERROR);
  EXPECT_NEAR(geometry_tools::to_radians(-180), -3.141592, ABS_ERROR);
}
