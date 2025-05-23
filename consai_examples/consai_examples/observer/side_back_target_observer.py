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

"""サイドバックターゲットを監視するモジュール."""

import math

from consai_examples.observer.field_normalizer import FieldNormalizer
from consai_examples.observer.field_positions import FieldPositions
from consai_examples.observer.pos_vel import PosVel

from consai_msgs.msg import State2D


class SideBackTargetObserver():
    """サイドバックターゲットを監視するクラス."""

    def __init__(self):
        """SideBackTargetObserverを初期化する関数."""
        # 0が上側、1が下側
        self._side_back_targets: dict[int, int] = {0: None, 1: None}

        self._field_pos = FieldPositions()
        self._field = FieldNormalizer()

    def _defense_area_margin(self) -> float:
        """ディフェンスエリアのマージンを取得する関数."""
        # FIXME: 要調整、ディフェンスエリア侵入が多いようなら大きくする
        return self._field.on_div_a_x(0.0)

    def _defense_area_front_margin(self) -> float:
        """ディフェンスエリア前方のマージンを取得する関数."""
        # FIXME: 要調整、ディフェンスエリア侵入が多いようなら大きくする
        return self._field.on_div_a_x(0.2)

    def set_field_positions(self, field_positions: FieldPositions) -> None:
        """フィールドポジションを設定する関数."""
        self._field_pos = field_positions

    def set_field_normalizer(self, field_normalizer: FieldNormalizer) -> None:
        """フィールドの正規化を設定する関数."""
        self._field = field_normalizer

    def has_target(self, side_id: int) -> bool:
        """指定されたサイドにターゲットがいるかを判定する関数."""
        return self._side_back_targets[side_id] is not None

    def get_target_id(self, side_id: int) -> int:
        """指定されたサイドのターゲットIDを取得する関数."""
        return self._side_back_targets[side_id]

    def update(self, their_robots: dict[int, PosVel]) -> None:
        """相手ロボット情報を更新する関数."""
        # ディフェンスエリア横にいる相手ロボットを検出する
        # 存在しない場合はNoneをセットする
        self._reset_targets()

        self._update_targets(their_robots)

    def _update_targets(self, their_robots: dict[int, PosVel]) -> None:
        """ターゲットを更新する関数（ゴールに近い相手ロボットを選定する）."""
        nearest_y0 = 10.0
        nearest_y1 = 10.0
        nearest_id0 = None
        nearest_id1 = None
        for robot_id, robot in their_robots.items():
            if self._is_in_top_side(robot.pos()):
                if math.fabs(robot.pos().y) < nearest_y0:
                    nearest_y0 = math.fabs(robot.pos().y)
                    nearest_id0 = robot_id
                continue

            if self._is_in_bottom_side(robot.pos()):
                if math.fabs(robot.pos().y) < nearest_y1:
                    nearest_y1 = math.fabs(robot.pos().y)
                    nearest_id1 = robot_id
                continue

        self._side_back_targets[0] = nearest_id0
        self._side_back_targets[1] = nearest_id1

    def _reset_targets(self) -> None:
        """ターゲット情報を初期化する関数."""
        self._side_back_targets = {0: None, 1: None}

    def _is_in_defense_area(self, pos: State2D) -> bool:
        """指定された位置がディフェンスエリアにあるかを判定する関数."""
        # ディフェンスエリアに入ってたらtrue
        penalty_upper_front = self._field_pos.penalty_pose('our', 'upper_front')
        defense_x = penalty_upper_front.x + self._defense_area_margin()
        defense_y = penalty_upper_front.y + self._defense_area_margin()
        if pos.x < defense_x and math.fabs(pos.y) < defense_y:
            return True
        return False

    def _is_in_top_side(self, pos: State2D) -> bool:
        """指定された位置がディフェンスエリア横の上側にあるかを判定する関数."""
        # ディフェンスエリアの横（上側）にロボットがいればtrue
        if self._is_in_defense_area(pos):
            return False

        penalty_upper_front = self._field_pos.penalty_pose('our', 'upper_front')

        if pos.x < (penalty_upper_front.x + self._defense_area_front_margin()) and \
                pos.y > penalty_upper_front.y:
            return True
        return False

    def _is_in_bottom_side(self, pos: State2D) -> bool:
        """指定された位置がディフェンスエリア横の下側にあるかを判定する関数."""
        # ディフェンスエリアの横（下側）にロボットがいればtrue
        if self._is_in_defense_area(pos):
            return False

        penalty_lower_front = self._field_pos.penalty_pose('our', 'lower_front')

        if pos.x < (penalty_lower_front.x + self._defense_area_front_margin()) and \
                pos.y < penalty_lower_front.y:
            return True
        return False
