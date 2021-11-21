#!/usr/bin/env python3
# coding: UTF-8

# Copyright 2021 Roots
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

from rclpy.node import Node
from robocup_ssl_msgs.msg import TrackedBall
from robocup_ssl_msgs.msg import TrackedFrame


# フィールド状況を観察し、ボールの位置を判断したり
# ロボットに一番近いロボットを判定する
class FieldObserver(Node):
    BALL_NONE = 0
    BALL_IS_OUTSIDE = 1
    BALL_IS_IN_OUR_DEFENSE_AREA = 2
    BALL_IS_IN_OUR_SIDE = 3
    BALL_IS_IN_THEIR_SIDE = 4
    BALL_IS_IN_THEIR_DEFENSE_AREA = 5

    def __init__(self):
        super().__init__('field_observer')

        self._ball_state = self.BALL_NONE

        self._field_length = 12.0  # meters
        self._field_half_length = self._field_length * 0.5
        self._field_width = 9.0  # meters
        self._field_half_width = self._field_width * 0.5
        self._field_defence_length = 1.8  # meters
        self._field_defence_width = 3.6  # meters
        self._field_defence_half_width = self._field_defence_width * 0.5  # meters
        self._sub_detection_tracked = self.create_subscription(
            TrackedFrame, 'detection_tracked', self._detection_tracked_callback, 10)

    def _detection_tracked_callback(self, msg):
        self._detection = msg
        if len(msg.balls) > 0:
            self._update_ball_state(msg.balls[0])

    def _update_ball_state(self, ball):
        # フィールド場外判定
        if math.fabs(ball.pos.x) > self._field_half_length or \
            math.fabs(ball.pos.y) > self._field_half_width:
            self._ball_state = self.BALL_IS_OUTSIDE
        else:
            self._ball_state = self.BALL_IS_IN_OUR_SIDE

    def get_ball_state(self):
        return self._ball_state

    def ball_is_outside(self):
        return self._ball_state == self.BALL_IS_OUTSIDE
