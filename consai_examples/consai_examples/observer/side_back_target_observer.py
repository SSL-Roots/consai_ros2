# Copyright 2024 Roots
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

from consai_examples.observer.pos_vel import PosVel
from consai_msgs.msg import State2D
from field import Field
import math


class SideBackTargetObserver():

    def __init__(self):
        # 0が上側、1が下側
        self._side_back_targets: dict[int, int] = {0: None, 1: None}
        # ロボット半径 x 2 + α
        self._DEFENSE_AREA_MARGIN = 0.6
        self._penalty_corner_upper_front = Field.penalty_pose('our', 'upper_front')
        self._penalty_corner_lower_front = Field.penalty_pose('our', 'lower_front')

    def has_target(self, side_id: int) -> bool:
        return self._side_back_targets[side_id] is not None

    def get_target_id(self, side_id: int) -> int:
        return self._side_back_targets[side_id]

    def update(self, their_robots: dict[int, PosVel]) -> None:
        # ディフェンスエリア横にいる相手ロボットを検出する
        # 存在しない場合はNoneをセットする
        self._reset_targets()

        self._update_targets(their_robots)

    def _update_targets(self, their_robots: dict[int, PosVel]) -> None:
        # ターゲットを抽出する
        nearest_x0 = 10.0
        nearest_x1 = 10.0
        nearest_id0 = None
        nearest_id1 = None
        for robot_id, robot in their_robots.items():
            if self._is_in_top_side(robot.pos()):
                if robot.pos().x < nearest_x0:
                    nearest_x0 = robot.pos().x
                    nearest_id0 = robot_id
                continue

            if self._is_in_bottom_side(robot.pos()):
                if robot.pos().x < nearest_x1:
                    nearest_x1 = robot.pos().x
                    nearest_id1 = robot_id
                continue

        self._side_back_targets[0] = nearest_id0
        self._side_back_targets[1] = nearest_id1

    def _reset_targets(self) -> None:
        # ターゲットを初期化する
        self._side_back_targets = {0: None, 1: None}

    def _is_in_defence_area(self, pos: State2D) -> bool:
        # ディフェンスエリアに入ってたらtrue
        defense_x = self._penalty_corner_upper_front.x + self._DEFENSE_AREA_MARGIN
        defense_y = self._penalty_corner_upper_front.y + self._DEFENSE_AREA_MARGIN
        if pos.x < defense_x and math.fabs(pos.y) < defense_y:
            return True
        return False

    def _is_in_top_side(self, pos: State2D) -> bool:
        # ディフェンスエリアの横（上側）にロボットがいればtrue
        if self._is_in_defence_area(pos):
            return False
        if pos.x < self._penalty_corner_upper_front.x and pos.y > self._penalty_corner_upper_front.y:
            return True
        return False

    def _is_in_bottom_side(self, pos: State2D) -> bool:
        # ディフェンスエリアの横（下側）にロボットがいればtrue
        if self._is_in_defence_area(pos):
            return False
        if pos.x < self._penalty_corner_lower_front.x and pos.y < self._penalty_corner_lower_front.y:
            return True
        return False
