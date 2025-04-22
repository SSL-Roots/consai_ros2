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

"""
キックターゲットを管理するモジュール.

シュートを試みるための最適なターゲット位置を計算し, キックターゲットの成功率を更新する.
"""

import numpy as np
from operator import attrgetter

from dataclasses import dataclass, field

from consai_msgs.msg import State2D

from consai_tools.geometry import geometry_tools as tool

from consai_game.utils.geometry import Point
from consai_game.world_model.ball_model import BallModel
from consai_game.world_model.field_model import Field
from consai_game.world_model.robots_model import Robot, RobotsModel


@dataclass
class KickTarget:
    """キックターゲットの位置と成功率を保持するデータクラス."""

    pos: State2D = field(default_factory=State2D)
    success_rate: int = 0


class KickTargetModel:
    """キックターゲットを保持するクラス."""

    def __init__(self):
        """KickTargetModelの初期化関数."""
        self.hysteresis_distance = 0.3
        self.robot_radius = 0.09

        self.shoot_target_list: list[KickTarget] = []
        self._goal_pos_list = [KickTarget()]

    def update_goal_pos_list(self, field: Field) -> None:
        """ゴール位置候補を更新する関数."""
        quarter_width = field.half_goal_width / 2
        one_eighth_width = field.half_goal_width / 4

        self._goal_pos_list = [
            KickTarget(pos=Point(field.half_length, 0.0)),
            KickTarget(pos=Point(field.half_length, one_eighth_width)),
            KickTarget(pos=Point(field.half_length, -one_eighth_width)),
            KickTarget(pos=Point(field.half_length, quarter_width)),
            KickTarget(pos=Point(field.half_length, -quarter_width)),
            KickTarget(pos=Point(field.half_length, quarter_width + one_eighth_width)),
            KickTarget(pos=Point(field.half_length, -(quarter_width + one_eighth_width))),
        ]

    def update(
        self,
        ball_model: BallModel,
        robots_model: RobotsModel,
    ) -> None:
        """キックターゲットを更新する関数."""
        self._ball = ball_model
        self._our_robots = robots_model.our_robots
        self._their_robots = robots_model.their_robots

        self.best_shoot_target = self._search_shoot_pos()

    def _update_scores(self, search_ours) -> list[KickTarget]:
        """各キックターゲットの成功率を計算し, リストを更新する関数."""
        TOLERANCE = self.robot_radius  # ロボット半径

        def obstacle_exists(target: State2D, robots: dict[int, Robot]) -> bool:
            """ターゲット位置に障害物（ロボット）が存在するかを判定する関数."""
            for robot in robots.values():
                if tool.is_on_line(robot.pos, self._ball.pos, target, TOLERANCE) and robot.is_visible:
                    return True
            return False

        for target in self._goal_pos_list:
            score = 0
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
                target.success_rate = int(score)

        return self._goal_pos_list

    def _sort_kick_targets_by_success_rate(self, targets: list[KickTarget]) -> list[KickTarget]:
        """スコアの高いターゲット順にソートする関数."""
        return sorted(targets, key=attrgetter("success_rate"), reverse=True)

    def _search_shoot_pos(self, search_ours=True) -> KickTarget:
        """ボールからの直線上にロボットがいないシュート位置リストを返す関数."""
        RATE_MARGIN = 50  # ヒステリシスのためのマージン
        last_shoot_target_list = self.shoot_target_list.copy()
        shoot_target_list = self._update_scores(search_ours)
        self.shoot_target_list = self._sort_kick_targets_by_success_rate(shoot_target_list)

        if not last_shoot_target_list:
            return self.shoot_target_list[0]

        if self.shoot_target_list[0].pos == last_shoot_target_list[0].pos:
            return self.shoot_target_list[0]

        # ヒステリシス処理
        if self.shoot_target_list[0].success_rate > last_shoot_target_list[0].success_rate + RATE_MARGIN:
            return self.shoot_target_list[0]
        return last_shoot_target_list[0]
