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


class ZoneManMarkTargetObserver():

    def __init__(self):
        self._zone_man_mark_targets: dict[int, int] = {0: None, 1: None, 2: None, 3: None}
        self._max_targets = 4

        # FIXME: 要調整、ディフェンスエリア侵入が多いようなら大きくする
        self._DEFENSE_AREA_MARGIN = 0.2
        self._penalty_corner_upper_front = Field.penalty_pose('our', 'upper_front')
        self._penalty_corner_lower_front = Field.penalty_pose('our', 'lower_front')

    def has_target(self, zone_id: int) -> bool:
        return self._zone_man_mark_targets[zone_id] is not None

    def get_target_id(self, zone_id: int) -> int:
        return self._zone_man_mark_targets[zone_id]

    def update(self, their_robots: dict[int, PosVel]) -> None:
        # ゾーンディフェンス担当者に合わせて、マンマークする相手ロボットを検出する
        # 存在しない場合はNoneをセットする
        self._reset_zone_targets()

        self._update_targets(their_robots)

    def _update_targets(self, their_robots: dict[int, PosVel]) -> None:
        # ZONE1に属するターゲットを抽出する
        nearest_x = 10.0

        # robot.pos().xの値とIDのタプルのリストを作成
        robot_x_id_pairs = [(robot.pos().x, robot_id) for robot_id, robot in their_robots.items()]

        # xがある値より小さい場合はそのロボットIDを無視する
        threshold_x = self._penalty_corner_upper_front.x + self._DEFENSE_AREA_MARGIN
        filtered_pairs = [(x, robot_id)
                          for x, robot_id in robot_x_id_pairs if (x < 0 and x > threshold_x)]

        # robot.pos().xの値で昇順ソート
        sorted_pairs = sorted(filtered_pairs)
        # ソート済みのロボットIDのリストを取得
        sorted_robot_ids = [robot_id for x, robot_id in sorted_pairs]

        # 空ではない
        if sorted_robot_ids:
            for i, sorted_robot_id in enumerate(sorted_robot_ids):
                if i >= self._max_targets:
                    break
                self._zone_man_mark_targets[i] = sorted_robot_id

    def _reset_zone_targets(self) -> None:
        # ZONEターゲットを初期化する
        self._zone_man_mark_targets = {0: None, 1: None, 2: None, 3: None}

    def _is_in_defence_area(self, pos: State2D) -> bool:
        # ディフェンスエリアに入ってたらtrue
        defense_x = self._penalty_corner_upper_front.x + self._DEFENSE_AREA_MARGIN

        # ディデンスエリア横はサイドバックが守るので無視
        if pos.x < defense_x:
            return True
        return False
