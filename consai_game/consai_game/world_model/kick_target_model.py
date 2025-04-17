# Copyright 2025 Roots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from dataclasses import dataclass, field

from consai_game.world_model.field_model import Field, FieldPoints
from consai_game.world_model.robots_model import Robot, RobotsModel
from consai_game.world_model.ball_model import BallModel
from consai_game.world_model.robot_activity_model import RobotActivityModel
from consai_tools.geometry import geometry_tools as tool
from consai_msgs.msg import State2D


@dataclass
class KickTarget:
    pos: State2D = field(default_factory=State2D)
    success_rate: int = 0


class KickTargetModel:
    """キックターゲットを保持するクラス."""

    def __init__(self, field: Field, field_points: FieldPoints):
        self.hysteresis_distance = 0.3
        self.robot_radius = 0.09
        self._goalie_id = 0

        self._field = field
        self._field_points = field_points

        self.target_point = self._field.half_goal_width / 2

        self._set_goal_pos_list()

        self._last_shoot_pos_list: list = None

    def _set_goal_pos_list(self) -> None:
        self._goal_pos_list = [
            State2D(self._field.half_length, 0.0),
            State2D(self._field.half_length, self.target_point),
            State2D(self._field.half_length, -self.target_point),
        ]

    def update(
        self,
        ball_model: BallModel,
        robots_model: RobotsModel,
        robot_activity_model: RobotActivityModel,
    ) -> None:
        """キックターゲットを更新する."""
        self._ball = ball_model
        self._our_robots = robots_model.our_robots
        self._their_robots = robots_model.their_robots
        self._our_visible_robots = robot_activity_model.our_visible_robots
        self._their_visible_robots = robot_activity_model.their_visible_robots

        self.best_shoot_target = self._search_shoot_pos()

    def _evaluate_shoot_target(self, target: State2D) -> int:
        score = 0

        # ゴール中央からの距離
        goal_center = State2D(x=self._field.half_length, y=0.0, theta=0.0)
        distance_from_center = tool.get_distance(target, goal_center)
        score += max(0, 20 - distance_from_center * 50)  # 0〜20点

        # ボールからの角度（目標方向がゴール方向と合っているか）
        angle = abs(tool.get_angle(self._ball.pos, target))
        score += max(0, 30 - angle * 20)  # 小さい角度（正面）ほど高得点

        # 相手ロボットによる遮蔽チェック（ライン上にいる数で評価を減点）
        blocked = 0
        for robot in self._their_robots.values():
            if tool.is_on_line(robot.pos, self._ball.pos, target, tolerance=0.1):
                blocked += 1
        score += max(0, 40 - blocked * 20)  # 最大2体まで考慮（0〜40点）

        # 距離（近いほうが成功率が高そう）
        distance = tool.get_distance(self._ball.pos, target)
        score += max(0, 10 - distance * 5)  # 距離2m以内ならOK

        return int(score)

    def _search_shoot_pos(self, search_ours=False) -> KickTarget:
        # ボールからの直線上にロボットがいないシュート位置リストを返す
        TOLERANCE = self.robot_radius  # ロボット半径

        shoot_pos_list = []

        def obstacle_exists(target: State2D, robots: dict[int, Robot]) -> bool:
            for robot in robots.values():
                if tool.is_on_line(robot.pos, self._ball.pos, target, TOLERANCE):
                    return True
            return False

        for target in self._goal_pos_list:
            if obstacle_exists(target, self._our_robots) and search_ours:
                continue
            if obstacle_exists(target, self._their_robots):
                continue
            shoot_pos_list.append(target)

        # ヒステリシス処理
        if self._last_shoot_pos_list is not None:
            last_shoot_pos = self._last_shoot_pos_list[0]
            still_valid = not obstacle_exists(last_shoot_pos, self._their_robots)
            if still_valid:
                shoot_pos_list.sort(key=lambda p: tool.get_distance(p, last_shoot_pos))
                if shoot_pos_list and tool.get_distance(shoot_pos_list[0], last_shoot_pos) < self.hysteresis_distance:
                    shoot_pos_list = [last_shoot_pos] + [p for p in shoot_pos_list if p != last_shoot_pos]

        self._last_shoot_pos_list = shoot_pos_list.copy() if shoot_pos_list else None

        success_rate = self._evaluate_shoot_target(shoot_pos_list[0])

        return KickTarget(pos=shoot_pos_list[0], success_rate=success_rate)
