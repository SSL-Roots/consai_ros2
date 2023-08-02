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

#include <gtest/gtest.h>
#include "consai_robot_controller/obstacle_ball.hpp"
#include "consai_robot_controller/obstacle_environment.hpp"
#include "consai_robot_controller/obstacle_robot.hpp"
#include "consai_robot_controller/prohibited_area.hpp"

TEST(TestObstacleEnvironment, handle_obstacle_robots) {
  obstacle::ObstacleEnvironment obstacle_environment;

  obstacle_environment.append_obstacle_robot(
    obstacle::ObstacleRobot(1.0, -2.0, 0.01));
  obstacle_environment.append_obstacle_robot(
    obstacle::ObstacleRobot(-3.0, 4.0, 0.02));

  EXPECT_EQ(obstacle_environment.get_obstacle_robots().size(), 2);
  EXPECT_EQ(obstacle_environment.get_obstacle_robots()[0].get_x(), 1.0);
  EXPECT_EQ(obstacle_environment.get_obstacle_robots()[0].get_y(), -2.0);
  EXPECT_EQ(obstacle_environment.get_obstacle_robots()[0].get_radius(), 0.01);

  EXPECT_EQ(obstacle_environment.get_obstacle_robots()[1].get_x(), -3.0);
  EXPECT_EQ(obstacle_environment.get_obstacle_robots()[1].get_y(), 4.0);
  EXPECT_EQ(obstacle_environment.get_obstacle_robots()[1].get_radius(), 0.02);

  obstacle_environment.clear_obstacle_robots();

  EXPECT_EQ(obstacle_environment.get_obstacle_robots().size(), 0);
}

TEST(TestObstacleEnvironment, handle_obstacle_balls) {
  obstacle::ObstacleEnvironment obstacle_environment;

  obstacle_environment.append_obstacle_ball(
    obstacle::ObstacleBall(1.0, -2.0, 0.01));
  obstacle_environment.append_obstacle_ball(
    obstacle::ObstacleBall(-3.0, 4.0, 0.02));

  EXPECT_EQ(obstacle_environment.get_obstacle_balls().size(), 2);
  EXPECT_EQ(obstacle_environment.get_obstacle_balls()[0].get_x(), 1.0);
  EXPECT_EQ(obstacle_environment.get_obstacle_balls()[0].get_y(), -2.0);
  EXPECT_EQ(obstacle_environment.get_obstacle_balls()[0].get_radius(), 0.01);

  EXPECT_EQ(obstacle_environment.get_obstacle_balls()[1].get_x(), -3.0);
  EXPECT_EQ(obstacle_environment.get_obstacle_balls()[1].get_y(), 4.0);
  EXPECT_EQ(obstacle_environment.get_obstacle_balls()[1].get_radius(), 0.02);

  obstacle_environment.clear_obstacle_balls();

  EXPECT_EQ(obstacle_environment.get_obstacle_balls().size(), 0);
}

TEST(TestObstacleEnvironment, handle_prohibited_areas) {
  obstacle::ObstacleEnvironment obstacle_environment;

  obstacle_environment.append_prohibited_area(
    obstacle::ProhibitedArea(1.0, -2.0, 3.0, -4.0));
  obstacle_environment.append_prohibited_area(
    obstacle::ProhibitedArea(-5.0, 6.0, -7.0, 8.0));

  EXPECT_EQ(obstacle_environment.get_prohibited_areas().size(), 2);
  EXPECT_EQ(obstacle_environment.get_prohibited_areas()[0].get_p1_x(), 1.0);
  EXPECT_EQ(obstacle_environment.get_prohibited_areas()[0].get_p1_y(), -2.0);
  EXPECT_EQ(obstacle_environment.get_prohibited_areas()[0].get_p2_x(), 3.0);
  EXPECT_EQ(obstacle_environment.get_prohibited_areas()[0].get_p2_y(), -4.0);
  EXPECT_EQ(obstacle_environment.get_prohibited_areas()[1].get_p1_x(), -5.0);
  EXPECT_EQ(obstacle_environment.get_prohibited_areas()[1].get_p1_y(), 6.0);
  EXPECT_EQ(obstacle_environment.get_prohibited_areas()[1].get_p2_x(), -7.0);
  EXPECT_EQ(obstacle_environment.get_prohibited_areas()[1].get_p2_y(), 8.0);

  obstacle_environment.clear_prohibited_areas();
  EXPECT_EQ(obstacle_environment.get_prohibited_areas().size(), 0);
}
