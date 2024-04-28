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
from consai_tools.geometry import geometry_tools as tool
import math


class BallPlacementObserver():
    def __init__(self):
        self._ball = PosVel()

        # FIXME JapanOpen2024用 パスを切るため
        self._threshold_far = 100.0  # meter
        self._threshold_arrived = 0.05  # meter
        self._threshold_arrived_velocity = 0.2  # m/s
        self._threshold_margin = 0.1  # meter
        self._threshold_margin_velocity = 2.0  # m/s

        self._prev_ball_is_arrived = False

    def update(self, ball: PosVel) -> None:
        self._ball = ball

    def is_far_from(self, placement_pos: State2D) -> bool:
        ball_pos = self._ball.pos()

        if tool.get_distance(ball_pos, placement_pos) > self._threshold_far:
            return True
        else:
            return False

    def is_arrived_at(self, placement_pos: State2D) -> bool:
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
