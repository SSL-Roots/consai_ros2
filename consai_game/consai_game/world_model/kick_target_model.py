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
import numpy as np


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

        self.half_half_goal_width = self._field.half_goal_width / 2
        self.quater_half_goal_width = self._field.half_goal_width / 4

        self._best_target_index = 0

    def update_goal_pos_list(self, field_points: FieldPoints) -> None:
        self._field_points = field_points
        self._goal_pos_list = [
            KickTarget(pos=Point(self._field.half_length, 0.0)),
            KickTarget(pos=Point(self._field.half_length, self.quater_half_goal_width)),
            KickTarget(pos=Point(self._field.half_length, -self.quater_half_goal_width)),
            KickTarget(pos=Point(self._field.half_length, self.half_half_goal_width)),
            KickTarget(pos=Point(self._field.half_length, -self.half_half_goal_width)),
            KickTarget(pos=Point(self._field.half_length, self.half_half_goal_width + self.quater_half_goal_width)),
            KickTarget(pos=Point(self._field.half_length, -(self.half_half_goal_width + self.quater_half_goal_width))),
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

    def _update_scores(self, search_ours) -> list[KickTarget]:
        score = 0
        TOLERANCE = self.robot_radius  # ロボット半径

        def obstacle_exists(target: State2D, robots: dict[int, Robot]) -> bool:
            for robot in robots.values():
                if tool.is_on_line(robot.pos, self._ball.pos, target, TOLERANCE) and robot.is_visible:
                    return True
            return False

        for target in self._goal_pos_list:
            if obstacle_exists(target.pos, self._our_robots) and search_ours:
                target.success_rate = 0
            elif obstacle_exists(target.pos, self._their_robots):
                target.success_rate = 0
            else:
                # ボールからの角度（目標方向がゴール方向と合っているか）
                angle = abs(tool.get_angle(self._ball.pos, target.pos))
                score += max(0, 60 - np.rad2deg(angle) * 2)  # 小さい角度（正面）ほど高得点

                # 距離（近いほうが成功率が高そう）
                distance = tool.get_distance(self._ball.pos, target.pos)
                score += max(0, 40 - distance * 20)  # 距離2m以内ならOK
                target.success_rate = score

        return self._goal_pos_list

    def _high_score_target_index(self, kick_target_list: KickTarget) -> int:
        # 最もスコアの高いターゲットのインデックスを返す
        high_score = 0
        high_score_index = 0
        for target in kick_target_list:
            if target.success_rate > high_score:
                high_score_index = kick_target_list.index(target)
        return high_score_index

    def _search_shoot_pos(self, search_ours=True) -> KickTarget:
        # ボールからの直線上にロボットがいないシュート位置リストを返す
        self.kick_target_list = self._update_scores(search_ours)

        high_score_target_index = self._high_score_target_index(self.kick_target_list)

        if high_score_target_index == self._best_target_index:
            return self.kick_target_list[high_score_target_index]

        # ヒステリシス処理
        if (
            self.kick_target_list[high_score_target_index].success_rate
            > self.kick_target_list[self._best_target_index].success_rate + 20
        ):
            self._best_target_index = high_score_target_index

        return self.kick_target_list[self._best_target_index]
