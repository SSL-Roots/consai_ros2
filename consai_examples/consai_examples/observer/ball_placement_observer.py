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

"""ボールの配置状態を監視するモジュール."""

import math

from consai_examples.observer.field_normalizer import FieldNormalizer
from consai_examples.observer.pos_vel import PosVel

from consai_msgs.msg import State2D

from consai_tools.geometry import geometry_tools as tool


class BallPlacementObserver():
    """ボールの配置状態を監視するクラス."""

    def __init__(self):
        """BallPlacementObserverを初期化する関数."""
        self._ball = PosVel()

        self._field = FieldNormalizer()

        self._prev_ball_is_arrived = False

    def set_field_normalizer(self, field_normalizer: FieldNormalizer) -> None:
        """フィールドの正規化を設定する関数."""
        self._field = field_normalizer

    def update(self, ball: PosVel) -> None:
        """ボールの位置と速度を更新する関数."""
        self._ball = ball

    def is_far_from(self, placement_pos: State2D) -> bool:
        """指定された位置からボールが遠いかどうかを判定する関数."""
        threshold_far = self._field.on_div_a_x(100.0)

        ball_pos = self._ball.pos()

        if tool.get_distance(ball_pos, placement_pos) > threshold_far:
            return True
        else:
            return False

    def is_arrived_at(self, placement_pos: State2D) -> bool:
        """指定された位置にボールが到達したかを判定する関数."""
        # FIXME JapanOpen2024用 パスを切るため
        self._threshold_arrived = self._field.on_div_a_x(0.05)  # meter
        self._threshold_arrived_velocity = self._field.on_div_a_x(0.2)  # m/s
        self._threshold_margin = self._field.on_div_a_x(0.1)  # meter
        self._threshold_margin_velocity = self._field.on_div_a_x(2.0)  # m/s

        ball_pos = self._ball.pos()
        ball_vel_norm = math.hypot(self._ball.vel().x, self._ball.vel().y)

        threshold = self._threshold_arrived
        threshold_vel = self._threshold_arrived_velocity
        if self._prev_ball_is_arrived:
            threshold += self._threshold_margin
            threshold_vel += self._threshold_margin_velocity

        if tool.get_distance(ball_pos, placement_pos) < threshold \
                and ball_vel_norm < threshold_vel:
            self._prev_ball_is_arrived = True
            return True
        else:
            self._prev_ball_is_arrived = False
            return False
