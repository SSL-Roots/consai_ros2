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

from consai_examples.observer.pos_vel import PosVel
from consai_msgs.msg import State2D
import math


class BallPositionStateObserver:
    def __init__(self):
        self._field_half_length = 6.0
        self._field_half_width = 4.5
        self._defense_area_length = 1.8
        self._defense_area_half_width = 1.8
        self._ball_pos = State2D()

    def update(self, ball_pos: State2D) -> None:
        self._ball_pos = ball_pos

    def is_outside_of_left(self) -> bool:
        if self._ball_pos.x < -self._field_half_length:
            return True

    def is_outside_of_right(self) -> bool:
        if self._ball_pos.x > self._field_half_length:
            return True

    def is_outside_of_top(self) -> bool:
        if self._ball_pos.y > self._field_half_width:
            return True

    def is_outside_of_bottom(self) -> bool:
        if self._ball_pos.y < -self._field_half_width:
            return True

    def is_outside(self) -> bool:
        return self.is_outside_of_left() or self.is_outside_of_right() or \
            self.is_outside_of_top() or self.is_outside_of_bottom()

    def is_in_our_defense_area(self):
        in_y = math.fabs(self._ball_pos.y) < self._defense_area_half_width
        in_x = self._ball_pos.x < -self._field_half_length + self._defense_area_length
        return in_y and in_x

    def is_in_their_defense_area(self):
        in_y = math.fabs(self._ball_pos.y) < self._defense_area_half_width
        in_x = self._ball_pos.x > self._field_half_length - self._defense_area_length
        return in_y and in_x

    def is_in_our_side(self):
        return self._ball_pos.x < 0

    def is_in_their_side(self):
        return self._ball_pos.x > 0
