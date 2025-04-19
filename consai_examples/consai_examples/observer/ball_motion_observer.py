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

import math

from consai_examples.observer.field_normalizer import FieldNormalizer
from consai_examples.observer.pos_vel import PosVel


class BallMotionObserver:
    def __init__(self):
        self._ball_is_moving = False

        self._field = FieldNormalizer()

    def set_field_normalizer(self, field_normalizer: FieldNormalizer) -> None:
        self._field = field_normalizer

    def update(self, ball: PosVel) -> None:
        self._update_ball_moving_state(ball)

    def is_moving(self) -> bool:
        return self._ball_is_moving

    def _update_ball_moving_state(self, ball: PosVel) -> None:
        # ボールが動いているか判定する
        BALL_MOVING_THRESHOLD = self._field.on_div_a_x(1.0)  # m/s
        BALL_MOVING_HYSTERESIS = self._field.on_div_a_x(0.3)  # m/s

        velocity_norm = math.hypot(ball.vel().x, ball.vel().y)

        # ボール速度がしきい値付近で揺れても、判定が切り替わらないようにヒステリシスを設ける
        threshold = BALL_MOVING_THRESHOLD
        if self._ball_is_moving:
            threshold = BALL_MOVING_THRESHOLD - BALL_MOVING_HYSTERESIS

        if velocity_norm > threshold:
            self._ball_is_moving = True
        else:
            self._ball_is_moving = False
