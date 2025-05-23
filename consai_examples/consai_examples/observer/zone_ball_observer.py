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

"""ボールがフィールド上のどのゾーンにあるかを監視するモジュール."""

from consai_examples.observer.field_normalizer import FieldNormalizer

from consai_msgs.msg import State2D


class ZoneBallObserver():
    """ボールがフィールド上のどのゾーンにあるかを監視するクラス."""

    BALL_ZONE_NONE = 0
    BALL_ZONE_LEFT_TOP = 1
    BALL_ZONE_LEFT_MID_TOP = 2
    BALL_ZONE_LEFT_MID_BOTTOM = 3
    BALL_ZONE_LEFT_BOTTOM = 4
    BALL_ZONE_RIGHT_TOP = 5
    BALL_ZONE_RIGHT_MID_TOP = 6
    BALL_ZONE_RIGHT_MID_BOTTOM = 7
    BALL_ZONE_RIGHT_BOTTOM = 8

    def __init__(self):
        """ボール位置とゾーン状態を設定する初期化関数."""
        self._ball_pos = State2D()
        self._ball_zone_state = self.BALL_ZONE_NONE

        self._field = FieldNormalizer()

    def set_field_normalizer(self, field_normalizer: FieldNormalizer) -> None:
        """フィールドの正規化設定をする関数."""
        self._field = field_normalizer

    def update(self, ball_pos: State2D, ball_is_in_our_side: bool) -> None:
        """ボールの位置と、ボールが自分側にあるかを更新する関数."""
        self._ball_pos = ball_pos
        self._update_ball_zone_state(ball_is_in_our_side)

    def ball_is_in_left_top(self) -> bool:
        """ボールが左上ゾーンにいるかを判定する関数."""
        return self._ball_zone_state == self.BALL_ZONE_LEFT_TOP

    def ball_is_in_left_mid_top(self) -> bool:
        """ボールが左中上ゾーンにいるかを判定する関数."""
        return self._ball_zone_state == self.BALL_ZONE_LEFT_MID_TOP

    def ball_is_in_left_mid_bottom(self) -> bool:
        """ボールが左中下ゾーンにいるかを判定する関数."""
        return self._ball_zone_state == self.BALL_ZONE_LEFT_MID_BOTTOM

    def ball_is_in_left_bottom(self) -> bool:
        """ボールが左下ゾーンにいるかを判定する関数."""
        return self._ball_zone_state == self.BALL_ZONE_LEFT_BOTTOM

    def ball_is_in_right_top(self) -> bool:
        """ボールが右上ゾーンにいるかを判定する関数."""
        return self._ball_zone_state == self.BALL_ZONE_RIGHT_TOP

    def ball_is_in_right_mid_top(self) -> bool:
        """ボールが右中上ゾーンにいるかを判定する関数."""
        return self._ball_zone_state == self.BALL_ZONE_RIGHT_MID_TOP

    def ball_is_in_right_mid_bottom(self) -> bool:
        """ボールが右中下ゾーンにいるかを判定する関数."""
        return self._ball_zone_state == self.BALL_ZONE_RIGHT_MID_BOTTOM

    def ball_is_in_right_bottom(self) -> bool:
        """ボールが右下ゾーンにいるかを判定する関数."""
        return self._ball_zone_state == self.BALL_ZONE_RIGHT_BOTTOM

    def ball_is_in_top(self) -> bool:
        """ボールが上部ゾーンにいるかを判定する関数."""
        return self._ball_zone_state in [
            self.BALL_ZONE_LEFT_TOP, self.BALL_ZONE_RIGHT_TOP,
            self.BALL_ZONE_LEFT_MID_TOP, self.BALL_ZONE_RIGHT_MID_TOP]

    def _update_ball_zone_state(self, ball_is_in_our_side: bool) -> None:
        """ボールがどのZONEに存在するのかを判定する関数."""
        ZONE_THRESHOLD = 0.2 * self._field.half_width() / self._field.on_div_a_y(4.5)
        field_quarter_width = self._field.half_width() * 0.5

        threshold_x = 0.0
        if ball_is_in_our_side:
            threshold_x += ZONE_THRESHOLD

        threshold_y_top = field_quarter_width
        if self.ball_is_in_right_top() or self.ball_is_in_left_top():
            threshold_y_top -= ZONE_THRESHOLD

        threshold_y_mid_top = 0.0
        if self.ball_is_in_right_mid_top() or self.ball_is_in_left_mid_top():
            threshold_y_mid_top -= ZONE_THRESHOLD

        threshold_y_mid_bottom = -field_quarter_width
        if self.ball_is_in_right_mid_bottom() or self.ball_is_in_left_mid_bottom():
            threshold_y_mid_bottom -= ZONE_THRESHOLD

        if self._ball_pos.x > threshold_x:
            if self._ball_pos.y > threshold_y_top:
                self._ball_zone_state = self.BALL_ZONE_RIGHT_TOP
            elif self._ball_pos.y > threshold_y_mid_top:
                self._ball_zone_state = self.BALL_ZONE_RIGHT_MID_TOP
            elif self._ball_pos.y > threshold_y_mid_bottom:
                self._ball_zone_state = self.BALL_ZONE_RIGHT_MID_BOTTOM
            else:
                self._ball_zone_state = self.BALL_ZONE_RIGHT_BOTTOM
        else:
            if self._ball_pos.y > threshold_y_top:
                self._ball_zone_state = self.BALL_ZONE_LEFT_TOP
            elif self._ball_pos.y > threshold_y_mid_top:
                self._ball_zone_state = self.BALL_ZONE_LEFT_MID_TOP
            elif self._ball_pos.y > threshold_y_mid_bottom:
                self._ball_zone_state = self.BALL_ZONE_LEFT_MID_BOTTOM
            else:
                self._ball_zone_state = self.BALL_ZONE_LEFT_BOTTOM
