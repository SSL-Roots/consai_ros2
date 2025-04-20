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

"""フィールドのサイズ, ロボットやボールの直径, DIV-Aサイズを正規化するクラスを提供するモジュール."""


class FieldNormalizer:
    """フィールドのサイズ, ロボットやボールの直径, DIV-Aサイズを正規化するクラス."""

    def __init__(self):
        """初期化関数, フィールドサイズ, ロボット直径, ボール直径, DIV-Aサイズを設定する関数."""
        self.set_field_size()
        self.set_robot_diameter()
        self.set_ball_diameter()
        self.set_div_a_size()

    def set_field_size(
            self, length=12.0, width=9.0, goal_width=1.8,
            penalty_depth=1.8, penalty_width=3.6):
        """フィールドサイズを設定する関数."""
        self._field_length = length
        self._field_width = width
        self._goal_width = goal_width
        self._penalty_depth = penalty_depth
        self._penalty_width = penalty_width

    def set_robot_diameter(self, diameter=0.18):
        """ロボットの直径を設定する関数."""
        self._robot_diameter = diameter

    def set_ball_diameter(self, diameter=0.043):
        """ボールの直径を設定する関数."""
        self._ball_diameter = diameter

    def set_div_a_size(
            self, length=12.0, width=9.0, robot_diameter=0.18, ball_diameter=0.043):
        """DIV-Aサイズを設定する関数."""
        self._div_a_length = length
        self._div_a_width = width
        self._div_a_robot_diameter = robot_diameter
        self._div_a_ball_diameter = ball_diameter

    def length(self) -> float:
        """フィールドの長さを返す関数."""
        return self._field_length

    def width(self) -> float:
        """フィールドの幅を返す関数."""
        return self._field_width

    def goal_width(self) -> float:
        """ゴールの幅を返す関数."""
        return self._goal_width

    def half_length(self) -> float:
        """フィールドの半分の長さを返す関数."""
        return self._field_length * 0.5

    def half_width(self) -> float:
        """フィールドの半分の幅を返す関数."""
        return self._field_width * 0.5

    def half_goal_width(self) -> float:
        """ゴールの半分の幅を返す関数."""
        return self._goal_width * 0.5

    def penalty_depth(self) -> float:
        """ペナルティエリアの奥行きを返す関数."""
        return self._penalty_depth

    def penalty_width(self) -> float:
        """ペナルティエリアの幅を返す関数."""
        return self._penalty_width

    def half_penalty_width(self) -> float:
        """ペナルティエリアの半分の幅を返す関数."""
        return self._penalty_width * 0.5

    def on_div_a_x(self, x: float) -> float:
        """X座標を正規化する関数."""
        return x * self._field_length / self._div_a_length

    def on_div_a_y(self, y: float) -> float:
        """Y座標を正規化する関数."""
        return y * self._field_width / self._div_a_width

    def on_div_a_robot_diameter(self, size: float) -> float:
        """ロボットの直径を正規化する関数."""
        return size * self._robot_diameter / self._div_a_robot_diameter

    def on_div_a_ball_diameter(self, size: float) -> float:
        """ボールの直径を正規化する関数."""
        return size * self._ball_diameter / self._div_a_ball_diameter
