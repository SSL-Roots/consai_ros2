#!/usr/bin/env python3
# coding: UTF-8

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
敵ロボットの驚異度を計算し、驚異度の高い順にソートするモジュール.

驚異度は敵ロボットのゴールへの距離, シュートできるか, ボールとの距離の3つの要素から計算される.
"""

from dataclasses import dataclass
from typing import Dict, List

from consai_msgs.msg import State2D

from consai_game.world_model.field_model import Field, FieldPoints
from consai_game.world_model.robots_model import RobotsModel
from consai_game.world_model.ball_model import BallModel

from consai_tools.geometry import geometry_tools as tools


@dataclass
class Threat:
    """敵ロボットの脅威度情報を保持するデータクラス"""

    score: int  # 0以上
    robot_id: int  # 相手のロボットID


class ThreatsEvaluation:
    """敵ロボットの脅威度を評価し順位付けするクラス"""

    ALPHA = 0.1  # ローパスフィルターの係数（0-1、小さいほど変化が遅い）

    def __init__(self, field: Field, field_points: FieldPoints):
        """TreatsEvalutionの初期化"""
        self.threats: List[Threat] = []
        self._field = field
        self._field_points = field_points
        self._prev_scores: Dict[int, float] = {}  # ロボットIDごとの前回のスコア

    def _apply_low_pass_filter(self, robot_id: int, new_score: float) -> float:
        """ローパスフィルターを適用してスコアを平滑化する

        Args:
            robot_id: ロボットID
            new_score: 新しいスコア

        Returns:
            平滑化されたスコア
        """
        if robot_id not in self._prev_scores:
            self._prev_scores[robot_id] = new_score
            return new_score

        # 前回のスコアと新しいスコアを重み付けして結合
        filtered_score = self._prev_scores[robot_id] * (1 - self.ALPHA) + new_score * self.ALPHA
        self._prev_scores[robot_id] = filtered_score
        return filtered_score

    def update(self, ball: BallModel, robots: RobotsModel):
        """敵ロボットの驚異度を更新する

        Args:
            ball: ボールの情報
            robots: ロボットのリスト
        """
        self.threats = []

        # 敵ロボットのみを対象とする（is_visibleがtrueのものだけ）
        for robot_id, robot in robots.their_robots.items():
            if not robot.is_visible:
                continue

            # A: ゴールへの距離を計算
            goal = State2D(x=self._field_points.our_goal_top.x, y=0.0)
            goal_distance = tools.get_distance(goal, robot.pos)

            # B: シュートできるか（ゴールとの間に障害物があるか）
            can_shoot = True
            for other_robot in robots.our_robots.values():
                if not other_robot.is_visible:
                    continue
                # ロボットとゴールを結ぶ直線上に他のロボットがいるかチェック
                if tools.is_on_line(
                    pose=other_robot.pos, line_pose1=robot.pos, line_pose2=goal, tolerance=0.1  # ロボットの半径を考慮
                ):
                    can_shoot = False
                    break

            # C: ボールとの距離を計算
            ball_distance = tools.get_distance(robot.pos, ball.pos)

            # 各要素のスコアを計算
            # A: ゴールへの距離（近いほど高スコア）
            max_distance = self._field.length
            score_a = int((max_distance - goal_distance) * 100 / max_distance)

            # B: シュートできるか（できる場合高スコア）
            score_b = 100 if can_shoot else 0

            # C: ボールとの距離（近いほど高スコア）
            max_ball_distance = self._field.length
            score_c = int((max_ball_distance - ball_distance) * 100 / max_ball_distance)

            # 総合スコアを計算
            # Bは一旦無視
            total_score = int(score_a * 0.8 + score_b * 0.0 + score_c * 0.2)

            # ローパスフィルターを適用
            filtered_score = self._apply_low_pass_filter(robot_id, total_score)

            threat = Threat(score=int(filtered_score), robot_id=robot_id)
            self.threats.append(threat)

        # スコアの高い順にソート
        self.threats.sort(key=lambda x: x.score, reverse=True)
