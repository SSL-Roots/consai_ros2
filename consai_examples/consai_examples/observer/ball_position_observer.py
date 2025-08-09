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

"""ボールの位置を監視するモジュール."""

import math

from consai_examples.observer.field_normalizer import FieldNormalizer

from consai_msgs.msg import State2D


class BallPositionObserver:
    """ボールの位置を監視するクラス."""

    def __init__(self):
        """BallPositionObserverを初期化する関数."""
        self._ball_pos = State2D()
        self._field = FieldNormalizer()

    def set_field_normalizer(self, field_normalizer: FieldNormalizer) -> None:
        """フィールドの正規化を設定する関数."""
        self._field = field_normalizer

    def _outside_margin(self) -> float:
        """外側マージンを取得する関数."""
        return self._field.on_div_a_ball_diameter(0.05)

    def update(self, ball_pos: State2D) -> None:
        """ボールの位置を更新する関数."""
        self._ball_pos = ball_pos

    def is_outside_of_left(self) -> bool:
        """ボールが左側外に出たかを判定する関数."""
        if self._ball_pos.x < -self._field.half_length():
            return True

    def is_outside_of_right(self) -> bool:
        """ボールが右側外に出たかを判定する関数."""
        if self._ball_pos.x > self._field.half_length():
            return True

    def is_outside_of_top(self) -> bool:
        """ボールが上側外に出たかを判定する関数."""
        if self._ball_pos.y > self._field.half_width():
            return True

    def is_outside_of_bottom(self) -> bool:
        """ボールが下側外に出たかを判定する関数."""
        if self._ball_pos.y < -self._field.half_width():
            return True

    def is_outside(self) -> bool:
        """ボールがフィールド外に出たかを判定する関数."""
        return self.is_outside_of_left() or self.is_outside_of_right() or \
            self.is_outside_of_top() or self.is_outside_of_bottom()

    def is_outside_of_left_with_margin(self) -> bool:
        """ボールが左側外にマージンを考慮して出たかを判定する関数."""
        if self._ball_pos.x < -self._field.half_length() - self._outside_margin():
            return True

    def is_outside_of_right_with_margin(self) -> bool:
        """ボールが右側外にマージンを考慮して出たかを判定する関数."""
        if self._ball_pos.x > self._field.half_length() + self._outside_margin():
            return True

    def is_outside_of_top_with_margin(self) -> bool:
        """ボールが上側外にマージンを考慮して出たかを判定する関数."""
        if self._ball_pos.y > self._field.half_width() + self._outside_margin():
            return True

    def is_outside_of_bottom_with_margin(self) -> bool:
        """ボールが下側外にマージンを考慮して出たかを判定する関数."""
        if self._ball_pos.y < -self._field.half_width() - self._outside_margin():
            return True

    def is_outside_with_margin(self) -> bool:
        """ボールがフィールド外にマージンを考慮して出たかを判定する関数."""
        return self.is_outside_of_left_with_margin() or \
            self.is_outside_of_right_with_margin() or \
            self.is_outside_of_top_with_margin() or \
            self.is_outside_of_bottom_with_margin()

    def is_in_our_defense_area(self):
        """ボールが自陣のディフェンスエリア内にあるかを判定する関数."""
        in_y = math.fabs(self._ball_pos.y) < self._field.half_penalty_width()
        in_x = self._ball_pos.x < -self._field.half_length() + self._field.penalty_depth()
        return in_y and in_x

    def is_in_their_defense_area(self):
        """ボールが相手陣のディフェンスエリア内にあるかを判定する関数."""
        in_y = math.fabs(self._ball_pos.y) < self._field.half_penalty_width()
        in_x = self._ball_pos.x > self._field.half_length() - self._field.penalty_depth()
        return in_y and in_x

    def is_in_our_side(self):
        """ボールが自陣にあるかを判定する関数."""
        return self._ball_pos.x < 0

    def is_in_their_side(self):
        """ボールが相手陣にあるかを判定する関数."""
        return self._ball_pos.x > 0
