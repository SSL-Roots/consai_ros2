// Copyright 2024 Roots
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
#include "consai_msgs/msg/state2_d.hpp"
#include "consai_tools/geometry_tools.hpp"

using State = consai_msgs::msg::State2D;

TEST(TestTools, calc_angle) {
  State from_pose;
  State to_pose;
  to_pose.x = 1.0;
  EXPECT_FLOAT_EQ(geometry_tools::calc_angle(from_pose, to_pose), 0.0);

  to_pose.x = 0.0;
  to_pose.y = 1.0;
  EXPECT_FLOAT_EQ(geometry_tools::calc_angle(from_pose, to_pose), M_PI_2);

  to_pose.x = -1.0;
  to_pose.y = 0.0;
  EXPECT_FLOAT_EQ(geometry_tools::calc_angle(from_pose, to_pose), M_PI);

  to_pose.x = 0.0;
  to_pose.y = -1.0;
  EXPECT_FLOAT_EQ(geometry_tools::calc_angle(from_pose, to_pose), -M_PI_2);
}

TEST(TestGeometryTools, is_lines_intersect)
{
  State p1, p2, p3, p4;
  auto line1_p1 = geometry_tools::gen_state(0.0, 0.0);
  auto line1_p2 = geometry_tools::gen_state(1.0, 1.0);
  auto line2_p1 = geometry_tools::gen_state(0.0, 1.0);
  auto line2_p2 = geometry_tools::gen_state(1.0, 0.0);
  EXPECT_TRUE(geometry_tools::is_lines_intersect(line1_p1, line1_p2, line2_p1, line2_p2));


  line2_p1 = geometry_tools::gen_state(0.0, 1.0);
  line2_p2 = geometry_tools::gen_state(1.0, 1.0);
  EXPECT_FALSE(geometry_tools::is_lines_intersect(line1_p1, line1_p2, line2_p1, line2_p2));
}
