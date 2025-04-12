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

from consai_msgs.msg import State2D
from consai_game.world_model.field_model import Field, FieldPoints
from consai_game.world_model.ball_model import BallModel
import math


class BallPositionModel:
    """ボールの位置情報を管理するクラス."""

    def __init__(self, field: Field, field_points: FieldPoints):
        self._pos = State2D()
        self._field = field
        self._field_points = field_points
        self._outside_margin = 0.05  # フィールド外判定のマージン

    def update_position(self, ball_model: BallModel, field_model: Field, field_points: FieldPoints) -> None:
        """ボールの位置を更新する."""
        self._pos = ball_model.pos
        self._field = field_model
        self._field_points = field_points

    def is_outside_of_left(self) -> bool:
        """ボールが左側のフィールド外にあるか判定する."""
        return self._pos.x < -self._field.length / 2

    def is_outside_of_right(self) -> bool:
        """ボールが右側のフィールド外にあるか判定する."""
        return self._pos.x > self._field.length / 2

    def is_outside_of_top(self) -> bool:
        """ボールが上側のフィールド外にあるか判定する."""
        return self._pos.y > self._field.width / 2

    def is_outside_of_bottom(self) -> bool:
        """ボールが下側のフィールド外にあるか判定する."""
        return self._pos.y < -self._field.width / 2

    def is_outside(self) -> bool:
        """ボールがフィールド外にあるか判定する."""
        return (self.is_outside_of_left() or self.is_outside_of_right() or
                self.is_outside_of_top() or self.is_outside_of_bottom())

    def is_outside_of_left_with_margin(self) -> bool:
        """マージンを考慮して、ボールが左側のフィールド外にあるか判定する."""
        return self._pos.x < -self._field.length / 2 - self._outside_margin

    def is_outside_of_right_with_margin(self) -> bool:
        """マージンを考慮して、ボールが右側のフィールド外にあるか判定する."""
        return self._pos.x > self._field.length / 2 + self._outside_margin

    def is_outside_of_top_with_margin(self) -> bool:
        """マージンを考慮して、ボールが上側のフィールド外にあるか判定する."""
        return self._pos.y > self._field.width / 2 + self._outside_margin

    def is_outside_of_bottom_with_margin(self) -> bool:
        """マージンを考慮して、ボールが下側のフィールド外にあるか判定する."""
        return self._pos.y < -self._field.width / 2 - self._outside_margin

    def is_outside_with_margin(self) -> bool:
        """マージンを考慮して、ボールがフィールド外にあるか判定する."""
        return (self.is_outside_of_left_with_margin() or
                self.is_outside_of_right_with_margin() or
                self.is_outside_of_top_with_margin() or
                self.is_outside_of_bottom_with_margin())

    def is_in_our_defense_area(self) -> bool:
        """ボールが自チームのディフェンスエリア内にあるか判定する."""
        in_y = math.fabs(self._pos.y) < self._field.penalty_width / 2
        in_x = self._pos.x < -self._field.length / 2 + self._field.penalty_depth
        return in_y and in_x

    def is_in_their_defense_area(self) -> bool:
        """ボールが相手チームのディフェンスエリア内にあるか判定する."""
        in_y = math.fabs(self._pos.y) < self._field.penalty_width / 2
        in_x = self._pos.x > self._field.length / 2 - self._field.penalty_depth
        return in_y and in_x

    def is_in_our_side(self) -> bool:
        """ボールが自チームのサイドにあるか判定する."""
        return self._pos.x < 0

    def is_in_their_side(self) -> bool:
        """ボールが相手チームのサイドにあるか判定する."""
        return self._pos.x > 0
