#!/usr/bin/env python3
# coding: UTF-8

"""
ボールの位置情報を管理するためのクラスを提供する.

ボールの位置更新, フィールド境界の判定（ヒステリシス付き）, および各エリア内かどうかの判定をする.
"""

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

import math

from consai_game.world_model.ball_model import BallModel
from consai_game.world_model.field_model import Field, FieldPoints

from consai_msgs.msg import State2D


class BallPositionModel:
    """ボールの位置情報を管理するクラス."""

    def __init__(self, field: Field, field_points: FieldPoints):
        """インスタンス生成時の初期化."""
        self._pos = State2D()
        self._field = field
        self._field_points = field_points
        self._outside_margin = 0.05  # フィールド外判定のマージン(m)
        self._hysteresis = 0.02  # ヒステリシスの閾値(m)
        self._last_left_state = False  # 前回の左側判定状態
        self._last_right_state = False  # 前回の右側判定状態
        self._last_top_state = False  # 前回の上側判定状態
        self._last_bottom_state = False  # 前回の下側判定状態
        self._last_our_defense_state = False  # 前回の自チームディフェンスエリア判定状態
        self._last_their_defense_state = False  # 前回の相手チームディフェンスエリア判定状態
        self._last_our_side_state = False  # 前回の自チームサイド判定状態
        self._last_their_side_state = False  # 前回の相手チームサイド判定状態

    def update_position(self, ball_model: BallModel, field_model: Field, field_points: FieldPoints) -> None:
        """ボールの位置を更新する."""
        self._pos = ball_model.pos
        self._field = field_model
        self._field_points = field_points

    def is_outside_of_left(self) -> bool:
        """ボールが左側のフィールド外にあるか判定する.ヒステリシス付き."""
        current_pos_state = self._pos.x < -self._field.half_length
        if current_pos_state != self._last_left_state:
            # 状態が変化する場合、ヒステリシスを考慮
            if current_pos_state:
                # 外に出た
                current_pos_state = self._pos.x < -self._field.half_length - self._hysteresis
            else:
                # 内に入った
                current_pos_state = self._pos.x < -self._field.half_length + self._hysteresis
        self._last_left_state = current_pos_state
        return current_pos_state

    def is_outside_of_right(self) -> bool:
        """ボールが右側のフィールド外にあるか判定する.ヒステリシス付き."""
        current_pos_state = self._pos.x > self._field.half_length
        if current_pos_state != self._last_right_state:
            # 状態が変化する場合、ヒステリシスを考慮
            if current_pos_state:
                # 外に出た
                current_pos_state = self._pos.x > self._field.half_length + self._hysteresis
            else:
                # 内に入った
                current_pos_state = self._pos.x > self._field.half_length - self._hysteresis
        self._last_right_state = current_pos_state
        return current_pos_state

    def is_outside_of_top(self) -> bool:
        """ボールが上側のフィールド外にあるか判定する.ヒステリシス付き."""
        current_pos_state = self._pos.y > self._field.half_width
        if current_pos_state != self._last_top_state:
            # 状態が変化する場合、ヒステリシスを考慮
            if current_pos_state:
                # 外に出た
                current_pos_state = self._pos.y > self._field.half_width + self._hysteresis
            else:
                # 内に入った
                current_pos_state = self._pos.y > self._field.half_width - self._hysteresis
        self._last_top_state = current_pos_state
        return current_pos_state

    def is_outside_of_bottom(self) -> bool:
        """ボールが下側のフィールド外にあるか判定する.ヒステリシス付き."""
        current_pos_state = self._pos.y < -self._field.half_width
        if current_pos_state != self._last_bottom_state:
            # 状態が変化する場合、ヒステリシスを考慮
            if current_pos_state:
                # 外に出た
                current_pos_state = self._pos.y < -self._field.half_width - self._hysteresis
            else:
                # 内に入った
                current_pos_state = self._pos.y < -self._field.half_width + self._hysteresis
        self._last_bottom_state = current_pos_state
        return current_pos_state

    def is_outside(self) -> bool:
        """ボールがフィールド外にあるか判定する."""
        return (
            self.is_outside_of_left()
            or self.is_outside_of_right()
            or self.is_outside_of_top()
            or self.is_outside_of_bottom()
        )

    def is_outside_of_left_with_margin(self) -> bool:
        """マージンを考慮して、ボールが左側のフィールド外にあるか判定する.ヒステリシス付き."""
        current_pos_state = self._pos.x < -self._field.half_length - self._outside_margin
        if current_pos_state != self._last_left_state:
            # 状態が変化する場合、ヒステリシスを考慮
            if current_pos_state:
                # 外に出た
                current_pos_state = self._pos.x < -self._field.half_length - self._outside_margin - self._hysteresis
            else:
                # 内に入った
                current_pos_state = self._pos.x < -self._field.half_length - self._outside_margin + self._hysteresis
        self._last_left_state = current_pos_state
        return current_pos_state

    def is_outside_of_right_with_margin(self) -> bool:
        """マージンを考慮して、ボールが右側のフィールド外にあるか判定する.ヒステリシス付き."""
        current_pos_state = self._pos.x > self._field.half_length + self._outside_margin
        if current_pos_state != self._last_right_state:
            # 状態が変化する場合、ヒステリシスを考慮
            if current_pos_state:
                # 外に出た
                current_pos_state = self._pos.x > self._field.half_length + self._outside_margin + self._hysteresis
            else:
                # 内に入った
                current_pos_state = self._pos.x > self._field.half_length + self._outside_margin - self._hysteresis
        self._last_right_state = current_pos_state
        return current_pos_state

    def is_outside_of_top_with_margin(self) -> bool:
        """マージンを考慮して、ボールが上側のフィールド外にあるか判定する.ヒステリシス付き."""
        current_pos_state = self._pos.y > self._field.half_width + self._outside_margin
        if current_pos_state != self._last_top_state:
            # 状態が変化する場合、ヒステリシスを考慮
            if current_pos_state:
                # 外に出た
                current_pos_state = self._pos.y > self._field.half_width + self._outside_margin + self._hysteresis
            else:
                # 内に入った
                current_pos_state = self._pos.y > self._field.half_width + self._outside_margin - self._hysteresis
        self._last_top_state = current_pos_state
        return current_pos_state

    def is_outside_of_bottom_with_margin(self) -> bool:
        """マージンを考慮して、ボールが下側のフィールド外にあるか判定する.ヒステリシス付き."""
        current_pos_state = self._pos.y < -self._field.half_width - self._outside_margin
        if current_pos_state != self._last_bottom_state:
            # 状態が変化する場合、ヒステリシスを考慮
            if current_pos_state:
                # 外に出た
                current_pos_state = self._pos.y < -self._field.half_width - self._outside_margin - self._hysteresis
            else:
                # 内に入った
                current_pos_state = self._pos.y < -self._field.half_width - self._outside_margin + self._hysteresis
        self._last_bottom_state = current_pos_state
        return current_pos_state

    def is_outside_with_margin(self) -> bool:
        """マージンを考慮して、ボールがフィールド外にあるか判定する."""
        return (
            self.is_outside_of_left_with_margin()
            or self.is_outside_of_right_with_margin()
            or self.is_outside_of_top_with_margin()
            or self.is_outside_of_bottom_with_margin()
        )

    def is_in_our_defense_area(self) -> bool:
        """ボールが自チームのディフェンスエリア内にあるか判定する.ヒステリシス付き."""
        current_pos_state = (
            math.fabs(self._pos.y) < self._field.half_penalty_width
            and self._pos.x < -self._field.half_length + self._field.penalty_depth
        )
        if current_pos_state != self._last_our_defense_state:
            # 状態が変化する場合、ヒステリシスを考慮
            if current_pos_state:
                # 内に入った
                current_pos_state = (
                    math.fabs(self._pos.y) < self._field.half_penalty_width - self._hysteresis
                    and self._pos.x < -self._field.half_length + self._field.penalty_depth - self._hysteresis
                )
            else:
                # 外に出た
                current_pos_state = (
                    math.fabs(self._pos.y) < self._field.half_penalty_width + self._hysteresis
                    and self._pos.x < -self._field.half_length + self._field.penalty_depth + self._hysteresis
                )
        self._last_our_defense_state = current_pos_state
        return current_pos_state

    def is_in_their_defense_area(self) -> bool:
        """ボールが相手チームのディフェンスエリア内にあるか判定する.ヒステリシス付き."""
        current_pos_state = (
            math.fabs(self._pos.y) < self._field.half_penalty_width
            and self._pos.x > self._field.half_length - self._field.penalty_depth
        )
        if current_pos_state != self._last_their_defense_state:
            # 状態が変化する場合、ヒステリシスを考慮
            if current_pos_state:
                # 内に入った
                current_pos_state = (
                    math.fabs(self._pos.y) < self._field.half_penalty_width - self._hysteresis
                    and self._pos.x > self._field.half_length - self._field.penalty_depth + self._hysteresis
                )
            else:
                # 外に出た
                current_pos_state = (
                    math.fabs(self._pos.y) < self._field.half_penalty_width + self._hysteresis
                    and self._pos.x > self._field.half_length - self._field.penalty_depth - self._hysteresis
                )
        self._last_their_defense_state = current_pos_state
        return current_pos_state

    def is_in_our_side(self) -> bool:
        """ボールが自チームのサイドにあるか判定する.ヒステリシス付き."""
        current_pos_state = self._pos.x < 0
        if current_pos_state != self._last_our_side_state:
            # 状態が変化する場合、ヒステリシスを考慮
            if current_pos_state:
                # 自チームサイドに入った
                current_pos_state = self._pos.x < -self._hysteresis
            else:
                # 自チームサイドから出た
                current_pos_state = self._pos.x < self._hysteresis
        self._last_our_side_state = current_pos_state

        # 相手チームサイドの判定と競合する場合は、相手チームサイドを優先
        if self._last_their_side_state:
            return False
        return current_pos_state

    def is_in_their_side(self) -> bool:
        """ボールが相手チームのサイドにあるか判定する.ヒステリシス付き."""
        current_pos_state = self._pos.x > 0
        if current_pos_state != self._last_their_side_state:
            # 状態が変化する場合、ヒステリシスを考慮
            if current_pos_state:
                # 相手チームサイドに入った
                current_pos_state = self._pos.x > self._hysteresis
            else:
                # 相手チームサイドから出た
                current_pos_state = self._pos.x > -self._hysteresis
        self._last_their_side_state = current_pos_state

        # 自チームサイドの判定と競合する場合は、相手チームサイドを優先
        if self._last_our_side_state:
            return False
        return current_pos_state
