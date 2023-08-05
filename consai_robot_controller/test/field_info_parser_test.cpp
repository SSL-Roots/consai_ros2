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
#include <memory>
#include "consai_msgs/action/robot_control.hpp"
#include "consai_msgs/msg/parsed_referee.hpp"
#include "consai_robot_controller/field_info_parser.hpp"
#include "robocup_ssl_msgs/msg/tracked_ball.hpp"
#include "robocup_ssl_msgs/msg/tracked_frame.hpp"
#include "robocup_ssl_msgs/msg/tracked_robot.hpp"


using RobotControl = consai_msgs::action::RobotControl;
using ParsedReferee = consai_msgs::msg::ParsedReferee;
using TrackedBall = robocup_ssl_msgs::msg::TrackedBall;
using TrackedFrame = robocup_ssl_msgs::msg::TrackedFrame;
using TrackedRobot = robocup_ssl_msgs::msg::TrackedRobot;

TEST(TestGetObstacleEnvironment, obstacle_ball) {
  auto goal = std::make_shared<RobotControl::Goal>();
  goal->avoid_obstacles = true;
  TrackedRobot my_robot;

  consai_robot_controller::FieldInfoParser field_info_parser;

  // ボール情報もレフェリー情報もないので、obstacle_ballは無い
  auto environment = field_info_parser.get_obstacle_environment(goal, my_robot);
  EXPECT_EQ(environment.get_obstacle_balls().size(), 0);

  // ボール情報を追加
  TrackedBall ball;
  ball.pos.x = 0.1;
  ball.pos.y = -2.3;
  ball.visibility.push_back(1.0);
  auto tracked_frame = std::make_shared<TrackedFrame>();
  tracked_frame->balls.push_back(ball);
  field_info_parser.set_detection_tracked(tracked_frame);

  // ボール情報を追加したが、レフェリー情報がないので、obstacle_ballは無い
  environment = field_info_parser.get_obstacle_environment(goal, my_robot);
  EXPECT_EQ(environment.get_obstacle_balls().size(), 0);

  // レフェリー情報を追加
  auto referee = std::make_shared<ParsedReferee>();
  referee->is_inplay = true;
  referee->is_our_setplay = false;
  field_info_parser.set_parsed_referee(referee);

  // inplayなのでobstacle_ballは無い
  environment = field_info_parser.get_obstacle_environment(goal, my_robot);
  EXPECT_EQ(environment.get_obstacle_balls().size(), 0);

  // our_setplayなのでobstacle_ballは無い
  referee->is_inplay = false;
  referee->is_our_setplay = true;
  field_info_parser.set_parsed_referee(referee);
  environment = field_info_parser.get_obstacle_environment(goal, my_robot);
  EXPECT_EQ(environment.get_obstacle_balls().size(), 0);

  // inplayでもour_setplayでもないのでobstacle_ballがある
  referee->is_inplay = false;
  referee->is_our_setplay = false;
  field_info_parser.set_parsed_referee(referee);
  environment = field_info_parser.get_obstacle_environment(goal, my_robot);
  EXPECT_EQ(environment.get_obstacle_balls().size(), 1);

  EXPECT_FLOAT_EQ(environment.get_obstacle_balls()[0].position().x(), 0.1);
  EXPECT_FLOAT_EQ(environment.get_obstacle_balls()[0].position().y(), -2.3);
  EXPECT_FLOAT_EQ(environment.get_obstacle_balls()[0].radius(), 0.0215);
}
