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
from consai_tools.geometry import geometry_tools as tool
from consai_msgs.msg import State2D
from consai_game.utils.geometry import Point


@dataclass
class KickTarget:
    pos: State2D = field(default_factory=State2D)
    success_rate: int = 0


class KickTargetModel:
    """キックターゲットを保持するクラス."""

    def __init__(self, field: Field, field_points: FieldPoints):
        self.hysteresis_distance = 0.3
        self.robot_radius = 0.09

        self._field = field
        self._field_points = field_points

        self.target_point = self._field.half_goal_width / 2

        self._set_goal_pos_list()

        self._last_best_shoot_pos = State2D()
        self._last_best_shoot_rate = 0

    def _set_goal_pos_list(self) -> None:
        self._goal_pos_list = [
            Point(self._field.half_length, 0.0),
            Point(self._field.half_length, self.target_point),
            Point(self._field.half_length, -self.target_point),
        ]

    def update(
        self,
        ball_model: BallModel,
        robots_model: RobotsModel,
    ) -> None:
        """キックターゲットを更新する."""
        self._ball = ball_model
        self._our_robots = robots_model.our_robots
        self._their_robots = robots_model.their_robots

        self.best_shoot_target = self._search_shoot_pos()

    def _evaluate_shoot_target(self, target: State2D) -> int:
        score = 0

        # ボールからの角度（目標方向がゴール方向と合っているか）
        angle = abs(tool.get_angle(self._ball.pos, target))
        score += max(0, 60 - angle * 2)  # 小さい角度（正面）ほど高得点

        # 距離（近いほうが成功率が高そう）
        distance = tool.get_distance(self._ball.pos, target)
        score += max(0, 40 - distance * 20)  # 距離2m以内ならOK

        return int(score)

    def _search_shoot_pos(self, search_ours=False) -> KickTarget:
        # ボールからの直線上にロボットがいないシュート位置リストを返す
        TOLERANCE = self.robot_radius * 2  # ロボット直径

        shoot_point_list = []

        def obstacle_exists(target: State2D, robots: dict[int, Robot]) -> bool:
            for robot in robots.values():
                if tool.is_on_line(robot.pos, self._ball.pos, target, TOLERANCE) and robot.is_visible:
                    return True
            return False

        for target in self._goal_pos_list:
            if obstacle_exists(target, self._our_robots) and search_ours:
                continue
            if obstacle_exists(target, self._their_robots):
                continue
            shoot_point_list.append(target)

        shoot_target = State2D()
        success_rate = 0
        best_shoot_target = State2D()
        best_success_rate = 0

        if shoot_point_list:
            for shoot_point in shoot_point_list:
                shoot_target.x = shoot_point.x
                shoot_target.y = shoot_point.y
                success_rate = self._evaluate_shoot_target(shoot_target)
                if success_rate > best_success_rate:
                    best_shoot_target = shoot_target
                    best_success_rate = success_rate

            # ヒステリシス処理
            if best_shoot_target != self._last_best_shoot_pos:
                for shoot_point in shoot_point_list:
                    if best_shoot_target == shoot_point:
                        best_shoot_target = shoot_point

            self._last_best_shoot_pos = best_shoot_target
            self._last_best_shoot_rate = best_success_rate

        return KickTarget(pos=best_shoot_target, success_rate=best_success_rate)
