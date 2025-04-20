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

"""ゾーンディフェンスのターゲットを観測・更新するモジュール."""

import math

from consai_examples.observer.field_normalizer import FieldNormalizer
from consai_examples.observer.pos_vel import PosVel

from consai_msgs.msg import State2D


class ZoneTargetObserver():
    """ゾーンディフェンスのターゲットを観測・更新するクラス."""

    def __init__(self):
        """ZoneTargetObserverクラスの初期化関数."""
        self._zone_targets: dict[int, int] = {0: None, 1: None, 2: None, 3: None}

        self._field = FieldNormalizer()

    def set_field_normalizer(self, field_normalizer: FieldNormalizer) -> None:
        """フィールド正規化のインスタンスを設定する関数."""
        self._field = field_normalizer

    def has_zone_target(self, zone_id: int) -> bool:
        """指定したゾーンIDにターゲットがいるかを確認する関数."""
        return self._zone_targets[zone_id] is not None

    def get_zone_target_id(self, zone_id: int) -> int:
        """指定したゾーンIDのターゲットIDを取得する関数."""
        return self._zone_targets[zone_id]

    def update(self, their_robots: dict[int, PosVel], num_of_zone_roles: int) -> None:
        """ゾーンディフェンス担当者に合わせて、マンマークする相手ロボットを検出する関数."""
        # 存在しない場合はNoneをセットする
        self._reset_zone_targets()

        if num_of_zone_roles == 1:
            self._update_zone1_targets(their_robots)
        elif num_of_zone_roles == 2:
            self._update_zone2_targets(their_robots)
        elif num_of_zone_roles == 3:
            self._update_zone3_targets(their_robots)
        elif num_of_zone_roles == 4:
            self._update_zone4_targets(their_robots)

    def _update_zone1_targets(self, their_robots: dict[int, PosVel]) -> None:
        """ZONE1に属するターゲットを抽出する関数."""
        nearest_x = 10.0
        nearest_id = None
        for robot_id, robot in their_robots.items():
            if self._is_in_zone1_0(robot.pos()):
                if robot.pos().x < nearest_x:
                    nearest_x = robot.pos().x
                    nearest_id = robot_id

        self._zone_targets[0] = nearest_id

    def _update_zone2_targets(self, their_robots: dict[int, PosVel]) -> None:
        """ZONE2に属するターゲットを抽出する関数."""
        nearest_x0 = 10.0
        nearest_x1 = 10.0
        nearest_id0 = None
        nearest_id1 = None
        for robot_id, robot in their_robots.items():
            if self._is_in_zone2_0(robot.pos()):
                if robot.pos().x < nearest_x0:
                    nearest_x0 = robot.pos().x
                    nearest_id0 = robot_id
                continue

            if self._is_in_zone2_1(robot.pos()):
                if robot.pos().x < nearest_x1:
                    nearest_x1 = robot.pos().x
                    nearest_id1 = robot_id
                continue

        self._zone_targets[0] = nearest_id0
        self._zone_targets[1] = nearest_id1

    def _update_zone3_targets(self, their_robots: dict[int, PosVel]) -> None:
        """ZONE3に属するターゲットを抽出する関数."""
        nearest_x0 = 10.0
        nearest_x1 = 10.0
        nearest_x2 = 10.0
        nearest_id0 = None
        nearest_id1 = None
        nearest_id2 = None
        for robot_id, robot in their_robots.items():
            if self._is_in_zone3_0(robot.pos()):
                if robot.pos().x < nearest_x0:
                    nearest_x0 = robot.pos().x
                    nearest_id0 = robot_id
                continue

            if self._is_in_zone3_1(robot.pos()):
                if robot.pos().x < nearest_x1:
                    nearest_x1 = robot.pos().x
                    nearest_id1 = robot_id
                continue

            if self._is_in_zone3_2(robot.pos()):
                if robot.pos().x < nearest_x2:
                    nearest_x2 = robot.pos().x
                    nearest_id2 = robot_id
                continue

        self._zone_targets[0] = nearest_id0
        self._zone_targets[1] = nearest_id1
        self._zone_targets[2] = nearest_id2

    def _update_zone4_targets(self, their_robots: dict[int, PosVel]) -> None:
        """ZONE4に属するターゲットを抽出する関数."""
        nearest_x0 = 10.0
        nearest_x1 = 10.0
        nearest_x2 = 10.0
        nearest_x3 = 10.0
        nearest_id0 = None
        nearest_id1 = None
        nearest_id2 = None
        nearest_id3 = None
        for robot_id, robot in their_robots.items():
            if self._is_in_zone4_0(robot.pos()):
                if robot.pos().x < nearest_x0:
                    nearest_x0 = robot.pos().x
                    nearest_id0 = robot_id
                continue

            if self._is_in_zone4_1(robot.pos()):
                if robot.pos().x < nearest_x1:
                    nearest_x1 = robot.pos().x
                    nearest_id1 = robot_id
                continue

            if self._is_in_zone4_2(robot.pos()):
                if robot.pos().x < nearest_x2:
                    nearest_x2 = robot.pos().x
                    nearest_id2 = robot_id
                continue

            if self._is_in_zone4_3(robot.pos()):
                if robot.pos().x < nearest_x3:
                    nearest_x3 = robot.pos().x
                    nearest_id3 = robot_id
                continue

        self._zone_targets[0] = nearest_id0
        self._zone_targets[1] = nearest_id1
        self._zone_targets[2] = nearest_id2
        self._zone_targets[3] = nearest_id3

    def _reset_zone_targets(self) -> None:
        """ZONEターゲットを初期化する関数."""
        self._zone_targets = {0: None, 1: None, 2: None, 3: None}

    def _is_in_defense_area(self, pos: State2D) -> bool:
        """ディフェンスエリアに入っているかを確認する関数."""
        # ディフェンスエリアに入ってたらtrue
        defense_x = self._field.on_div_a_x(-6.0 + 1.8) \
            + self._field.on_div_a_robot_diameter(0.6)  # 0.4はロボットの直径x2
        defense_y = self._field.on_div_a_y(1.8) \
            + self._field.on_div_a_robot_diameter(0.6)  # 0.4はロボットの直径x2
        if pos.x < defense_x and math.fabs(pos.y) < defense_y:
            return True
        return False

    def _is_in_zone1_0(self, pos: State2D) -> bool:
        """ZONE1 (左サイドの全部)にロボットがいればtrueを返す関数."""
        if self._is_in_defense_area(pos):
            return False

        if pos.x < 0.0:
            return True
        return False

    def _is_in_zone2_0(self, pos: State2D) -> bool:
        """ZONE2 (左サイドの上半分)にロボットがいればtrueを返す関数."""
        if self._is_in_defense_area(pos):
            return False
        if pos.x < 0.0 and pos.y > 0.0:
            return True
        return False

    def _is_in_zone2_1(self, pos: State2D) -> bool:
        """ZONE2 (左サイドの下半分)にロボットがいればtrueを返す関数."""
        if self._is_in_defense_area(pos):
            return False
        if pos.x < 0.0 and pos.y <= 0.0:
            return True
        return False

    def _is_in_zone3_0(self, pos: State2D) -> bool:
        """ZONE3 (左サイドの上半分の上)にロボットがいればtrueを返す関数."""
        if self._is_in_defense_area(pos):
            return False
        if pos.x < 0.0 and pos.y > self._field.on_div_a_y(4.5 * 0.5):
            return True
        return False

    def _is_in_zone3_1(self, pos: State2D) -> bool:
        """ZONE3 (左サイドの真ん中)にロボットがいればtrueを返す関数."""
        if self._is_in_defense_area(pos):
            return False
        if pos.x < 0.0 and math.fabs(pos.y) <= self._field.on_div_a_y(4.5 * 0.5):
            return True
        return False

    def _is_in_zone3_2(self, pos: State2D) -> bool:
        """ZONE3 (左サイドの下半分の下)にロボットがいればtrueを返す関数."""
        if self._is_in_defense_area(pos):
            return False
        if pos.x < 0.0 and pos.y < self._field.on_div_a_y(-4.5 * 0.5):
            return True
        return False

    def _is_in_zone4_0(self, pos: State2D) -> bool:
        """ZONE4 (左サイドの上半分の上)にロボットがいればtrueを返す関数."""
        if self._is_in_defense_area(pos):
            return False
        if pos.x < 0.0 and pos.y > self._field.on_div_a_y(4.5 * 0.5):
            return True
        return False

    def _is_in_zone4_1(self, pos: State2D) -> bool:
        """ZONE4 (左サイドの上半分の上)にロボットがいればtrueを返す関数."""
        if self._is_in_defense_area(pos):
            return False
        if pos.x < 0.0 and pos.y > 0.0 and pos.y <= self._field.on_div_a_y(4.5 * 0.5):
            return True
        return False

    def _is_in_zone4_2(self, pos: State2D) -> bool:
        """ZONE4 (左サイドの下半分の上)にロボットがいればtrueを返す関数."""
        if self._is_in_defense_area(pos):
            return False
        if pos.x < 0.0 and pos.y <= 0.0 and pos.y > self._field.on_div_a_y(-4.5 * 0.5):
            return True
        return False

    def _is_in_zone4_3(self, pos: State2D) -> bool:
        """ZONE4 (左サイドの下半分の下)にロボットがいればtrueを返す関数."""
        if self._is_in_defense_area(pos):
            return False
        if pos.x < 0.0 and pos.y <= self._field.on_div_a_y(-4.5 * 0.5):
            return True
        return False
