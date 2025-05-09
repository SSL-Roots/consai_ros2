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

#include <algorithm>
#include <memory>
#include <vector>

#include "consai_robot_controller/detection_extractor.hpp"
#include "robocup_ssl_msgs/msg/robot_id.hpp"

namespace parser
{

using RobotId = robocup_ssl_msgs::msg::RobotId;

DetectionExtractor::DetectionExtractor(const double visibility_threshold)
{
  visibility_threshold_ = visibility_threshold;
}

bool DetectionExtractor::extract_robot(
  const unsigned int robot_id, const bool team_is_yellow,
  TrackedRobot & my_robot) const
{
  if (!detection_tracked_) {
    return false;
  }
  // detection_trackedから指定された色とIDのロボット情報を抽出する
  // visibilityが低いときは情報が無いと判定する
  for (const auto & robot : detection_tracked_->robots) {
    if (robot_id != robot.robot_id.id) {
      continue;
    }
    const bool is_yellow =
      team_is_yellow && robot.robot_id.team_color == RobotId::TEAM_COLOR_YELLOW;
    const bool is_blue =
      !team_is_yellow && robot.robot_id.team_color == RobotId::TEAM_COLOR_BLUE;

    if (!is_yellow && !is_blue) {
      continue;
    }
    if (robot.visibility.size() == 0) {
      return false;
    }
    if (robot.visibility[0] < visibility_threshold_) {
      return false;
    }

    my_robot = robot;
    return true;
  }
  return false;
}

bool DetectionExtractor::extract_ball(TrackedBall & my_ball) const
{
  if (!detection_tracked_) {
    return false;
  }
  // detection_trackedからボール情報を抽出する
  // visibilityが低いときは情報が無いと判定する
  for (const auto & ball : detection_tracked_->balls) {
    if (ball.visibility.size() == 0) {
      return false;
    }
    if (ball.visibility[0] < visibility_threshold_) {
      return false;
    }

    my_ball = ball;
    break;
  }
  return true;
}

std::optional<TrackedBall> DetectionExtractor::extract_ball() const
{
  TrackedBall my_ball;
  if (extract_ball(my_ball)) {
    return my_ball;
  }
  return std::nullopt;
}

std::vector<TrackedRobot> DetectionExtractor::extract_robots() const
{
  if (!detection_tracked_) {
    return std::vector<TrackedRobot>();
  }

  std::vector<TrackedRobot> robots;
  for (const auto & robot : detection_tracked_->robots) {
    if (robot.visibility.size() == 0) {
      continue;
    }
    if (robot.visibility[0] < visibility_threshold_) {
      continue;
    }

    robots.push_back(robot);
  }
  return robots;
}

std::vector<unsigned int> DetectionExtractor::active_robot_id_list(const bool team_is_yellow) const
{
  if (!detection_tracked_) {
    return std::vector<unsigned int>();
  }

  // 存在しているロボットのIDリストを返す
  std::vector<unsigned int> id_list;
  for (const auto & robot : detection_tracked_->robots) {
    bool is_yellow = team_is_yellow && robot.robot_id.team_color == RobotId::TEAM_COLOR_YELLOW;
    bool is_blue = !team_is_yellow && robot.robot_id.team_color == RobotId::TEAM_COLOR_BLUE;
    if (!is_yellow && !is_blue) {
      continue;
    }

    if (robot.visibility.size() == 0) {
      continue;
    }
    if (robot.visibility[0] < visibility_threshold_) {
      continue;
    }

    id_list.push_back(robot.robot_id.id);
  }
  return id_list;
}

}  // namespace parser
