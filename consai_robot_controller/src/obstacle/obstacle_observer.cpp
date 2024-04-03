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


#include "consai_robot_controller/obstacle/obstacle_observer.hpp"
#include "consai_robot_controller/obstacle/obstacle_ball.hpp"
#include "consai_robot_controller/obstacle/obstacle_environment.hpp"
#include "consai_robot_controller/obstacle/obstacle_robot.hpp"
#include "consai_robot_controller/obstacle/obstacle_typedef.hpp"
#include "consai_robot_controller/obstacle/prohibited_area.hpp"
#include "robocup_ssl_msgs/msg/tracked_ball.hpp"

namespace obstacle
{

using ObstArea = obstacle::ProhibitedArea;
using ObstBall = obstacle::ObstacleBall;
using ObstEnv = obstacle::ObstacleEnvironment;
using ObstPos = obstacle::Position;
using ObstRadius = obstacle::Radius;
using ObstRobot = obstacle::ObstacleRobot;
using TrackedBall = robocup_ssl_msgs::msg::TrackedBall;


ObstacleObserver::ObstacleObserver(
  const std::shared_ptr<parser::DetectionExtractor> & detection_extractor)
{
  detection_ = detection_extractor;
}

ObstacleEnvironment ObstacleObserver::get_obstacle_environment(
  const RobotControlMsg::SharedPtr goal,
  const TrackedRobot & my_robot) const
{
  // constexpr ObstRadius BALL_RADIUS(0.0215);
  constexpr ObstRadius ROBOT_RADIUS(0.09);

  // goalから障害物環境を作成する
  ObstEnv environment;

  // 衝突回避しない場合は空の環境を返す
  if (!goal->avoid_obstacles) {
    return environment;
  }

  // インプレイと自チームセットプレイ以外ではボールから離れる
  // TODO(ShotaAk): ここは戦略側で判断できそう
  // if (parsed_referee_) {
  //   if (parsed_referee_->is_our_setplay == false && parsed_referee_->is_inplay == false) {
  //     TrackedBall ball;
  //     if (detection_->extract_ball(ball)) {
  //       environment.append_obstacle_ball(ObstBall(ObstPos(ball.pos.x, ball.pos.y), BALL_RADIUS));
  //     }
  //   }
  // }

  // ロボットを障害物として扱う
  for (const auto & robot : detection_->extract_robots()) {
    // 自身の情報は除外する
    if (robot.robot_id.id == my_robot.robot_id.id &&
      robot.robot_id.team_color == my_robot.robot_id.team_color)
    {
      continue;
    }

    environment.append_obstacle_robot(
      ObstRobot(ObstPos(robot.pos.x, robot.pos.y), ROBOT_RADIUS));
  }

  // TODO(ShotaAk): ボールプレースメント回避エリアとディフェンスエリアを障害物に追加する

  return environment;
}


}  // namespace obstacle
