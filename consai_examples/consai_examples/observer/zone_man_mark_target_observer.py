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

"""ゾーンディフェンスのターゲットを監視し, マンマーク対象を決定するクラスを提供するモジュール."""

from consai_examples.observer.field_normalizer import FieldNormalizer
from consai_examples.observer.field_positions import FieldPositions
from consai_examples.observer.pos_vel import PosVel

from consai_msgs.msg import State2D


class ZoneManMarkTargetObserver():
    """ゾーンディフェンスのターゲットを監視し, マンマーク対象を決定するクラス."""

    def __init__(self):
        """ゾーンディフェンスのターゲットを初期化する関数."""
        self._zone_man_mark_targets: dict[int, int] = {0: None, 1: None, 2: None, 3: None}
        self._max_targets = 4

        self._field_pos = FieldPositions()
        self._field = FieldNormalizer()

    def _defense_area_margin(self) -> float:
        """ディフェンスエリアのマージンを返す関数."""
        # FIXME: 要調整、ディフェンスエリア侵入が多いようなら大きくする
        return self._field.on_div_a_x(0.2)

    def set_field_positions(self, field_positions: FieldPositions) -> None:
        """フィールドの位置情報を設定する関数."""
        self._field_pos = field_positions

    def set_field_normalizer(self, field_normalizer: FieldNormalizer) -> None:
        """フィールドの正規化設定を行う関数."""
        self._field = field_normalizer

    def has_target(self, zone_id: int) -> bool:
        """指定されたゾーンIDにターゲットがいるか確認する関数."""
        return self._zone_man_mark_targets[zone_id] is not None

    def get_target_id(self, zone_id: int) -> int:
        """指定されたゾーンIDに対応するターゲットのIDを返す関数."""
        return self._zone_man_mark_targets[zone_id]

    def update(self, their_robots: dict[int, PosVel]) -> None:
        """ゾーンディフェンス担当者に合わせて、マンマークする相手ロボットを検出する関数."""
        # 存在しない場合はNoneをセットする
        self._reset_zone_targets()

        self._update_targets(their_robots)

    def _update_targets(self, their_robots: dict[int, PosVel]) -> None:
        """相手ロボットの位置に基づいて、ゾーンディフェンスターゲットを更新する関数."""
        # robot.pos().xの値とIDのタプルのリストを作成
        robot_x_id_pairs = [(robot.pos().x, robot_id) for robot_id, robot in their_robots.items()]

        # xがある値より小さい場合はそのロボットIDを無視する
        threshold_x = \
            self._field_pos.penalty_pose('our', 'upper_front').x + self._defense_area_margin()
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
        """ZONEターゲットを初期化する関数."""
        self._zone_man_mark_targets = {0: None, 1: None, 2: None, 3: None}

    def _is_in_defense_area(self, pos: State2D) -> bool:
        """ロボットがディフェンスエリア内にいるかを判定する関数."""
        # ディフェンスエリアに入ってたらtrue
        defense_x = \
            self._field_pos.penalty_pose('our', 'upper_front').x + self._defense_area_margin()

        # ディデンスエリア横はサイドバックが守るので無視
        if pos.x < defense_x:
            return True
        return False
