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
#include "consai_robot_controller/obstacle_typedef.hpp"
#include "consai_robot_controller/prohibited_area.hpp"

using ObstArea = obstacle::ProhibitedArea;
using ObstBall = obstacle::ObstacleBall;
using ObstEnv = obstacle::ObstacleEnvironment;
using ObstPos = obstacle::Position;
using ObstRadius = obstacle::Radius;
using ObstRobot = obstacle::ObstacleRobot;

TEST(TestObstacleEnvironment, handle_obstacle_robots) {
  ObstEnv obstacle_environment;

  ObstRobot robot1(ObstPos(1.0, -2.0), ObstRadius(0.01));
  ObstRobot robot2(ObstPos(-3.0, 4.0), ObstRadius(0.02));

  obstacle_environment.append_obstacle_robot(robot1);
  obstacle_environment.append_obstacle_robot(robot2);

  EXPECT_EQ(obstacle_environment.get_obstacle_robots().size(), 2);
  EXPECT_EQ(obstacle_environment.get_obstacle_robots()[0].position().x(), 1.0);
  EXPECT_EQ(obstacle_environment.get_obstacle_robots()[0].position().y(), -2.0);
  EXPECT_EQ(obstacle_environment.get_obstacle_robots()[0].radius(), 0.01);

  EXPECT_EQ(obstacle_environment.get_obstacle_robots()[1].position().x(), -3.0);
  EXPECT_EQ(obstacle_environment.get_obstacle_robots()[1].position().y(), 4.0);
  EXPECT_EQ(obstacle_environment.get_obstacle_robots()[1].radius(), 0.02);

  obstacle_environment.clear_obstacle_robots();

  EXPECT_EQ(obstacle_environment.get_obstacle_robots().size(), 0);
}

TEST(TestObstacleEnvironment, handle_obstacle_balls) {
  ObstEnv obstacle_environment;

  ObstBall ball1(ObstPos(1.0, -2.0), ObstRadius(0.01));
  ObstBall ball2(ObstPos(-3.0, 4.0), ObstRadius(0.02));

  EXPECT_EQ(obstacle_environment.get_obstacle_balls().size(), 2);
  EXPECT_EQ(obstacle_environment.get_obstacle_balls()[0].position().x(), 1.0);
  EXPECT_EQ(obstacle_environment.get_obstacle_balls()[0].position().y(), -2.0);
  EXPECT_EQ(obstacle_environment.get_obstacle_balls()[0].radius(), 0.01);

  EXPECT_EQ(obstacle_environment.get_obstacle_balls()[1].position().x(), -3.0);
  EXPECT_EQ(obstacle_environment.get_obstacle_balls()[1].position().y(), 4.0);
  EXPECT_EQ(obstacle_environment.get_obstacle_balls()[1].radius(), 0.02);

  obstacle_environment.clear_obstacle_balls();

  EXPECT_EQ(obstacle_environment.get_obstacle_balls().size(), 0);
}

TEST(TestObstacleEnvironment, handle_prohibited_areas) {
  ObstEnv obstacle_environment;

  ObstArea area1(ObstPos(1.0, -2.0), ObstPos(3.0, -4.0));
  ObstArea area2(ObstPos(-5.0, 6.0), ObstPos(-7.0, 8.0));

  obstacle_environment.append_prohibited_area(area1);
  obstacle_environment.append_prohibited_area(area2);

  EXPECT_EQ(obstacle_environment.get_prohibited_areas().size(), 2);
  EXPECT_EQ(obstacle_environment.get_prohibited_areas()[0].p1().x(), 1.0);
  EXPECT_EQ(obstacle_environment.get_prohibited_areas()[0].p1().y(), -2.0);
  EXPECT_EQ(obstacle_environment.get_prohibited_areas()[0].p2().x(), 3.0);
  EXPECT_EQ(obstacle_environment.get_prohibited_areas()[0].p2().y(), -4.0);
  EXPECT_EQ(obstacle_environment.get_prohibited_areas()[1].p1().x(), -5.0);
  EXPECT_EQ(obstacle_environment.get_prohibited_areas()[1].p1().y(), 6.0);
  EXPECT_EQ(obstacle_environment.get_prohibited_areas()[1].p2().x(), -7.0);
  EXPECT_EQ(obstacle_environment.get_prohibited_areas()[1].p2().y(), 8.0);

  obstacle_environment.clear_prohibited_areas();
  EXPECT_EQ(obstacle_environment.get_prohibited_areas().size(), 0);
}
