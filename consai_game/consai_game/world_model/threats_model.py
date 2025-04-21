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

from dataclasses import dataclass
from typing import List

from consai_msgs.msg import State2D
from consai_game.world_model.field_model import Field, FieldPoints
from consai_game.world_model.robots_model import RobotsModel
from consai_tools.geometry import geometry_tools as tools


@dataclass
class Threat:
    score: int  # 0以上
    robot_id: int  # 相手のロボットID


class ThreatsModel:
    def __init__(self, field: Field, field_points: FieldPoints):
        self.threats: List[Threat] = []
        self._field = field
        self._field_points = field_points

    def update(self, robots: RobotsModel):
        """敵ロボットの驚異度を更新する

        Args:
            robots: ロボットのリスト
        """
        self.threats = []

        # 敵ロボットのみを対象とする（is_visibleがtrueのものだけ）
        for robot_id, robot in robots.their_robots.items():
            if not robot.is_visible:
                continue

            # Aゴールへの距離を計算
            goal = State2D(x=self._field_points.our_goal_top.x, y=0.0)
            distance = tools.get_distance(goal, robot.pos)

            # 距離が近いほどスコアが高くなるように計算
            # フィールドの長さを基準にスコアを計算
            max_distance = self._field.length
            score = int((max_distance - distance) * 100 / max_distance)

            threat = Threat(score=score, robot_id=robot_id)
            self.threats.append(threat)

        # スコアの高い順にソート
        self.threats.sort(key=lambda x: x.score, reverse=True)
