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

    BALL_PLACEMENT_NONE = 0
    BALL_PLACEMENT_FAR_FROM_TARGET = 1
    BALL_PLACEMENT_NEAR_TARGET = 2
    BALL_PLACEMENT_ARRIVED_AT_TARGET = 3

    THRESHOLD_MARGIN = 0.1  # meters. 状態変化のしきい値にヒステリシスをもたせる

    def __init__(self):
        super().__init__('field_observer')

        self._ball_state = self.BALL_NONE
        self._ball_placement_state = self.BALL_PLACEMENT_NONE
        self._ball_is_moving = False
        self._ball = TrackedBall()

        self._field_x = 12.0  # meters
        self._field_half_x = self._field_x * 0.5
        self._field_y = 9.0  # meters
        self._field_half_y = self._field_y * 0.5
        self._field_defense_x = 1.8  # meters
        self._field_defense_y = 3.6  # meters
        self._field_defense_half_y = self._field_defense_y * 0.5  # meters
        self._sub_detection_tracked = self.create_subscription(
            TrackedFrame, 'detection_tracked', self._detection_tracked_callback, 10)

    def _detection_tracked_callback(self, msg):
        self._detection = msg
        if len(msg.balls) > 0:
            self._update_ball_state(msg.balls[0])
            self._update_ball_moving_state(msg.balls[0])
            self._ball = msg.balls[0]

    def _update_ball_state(self, ball):
        # フィールド場外判定
        if self._check_is_ball_outside(ball.pos):
            self._ball_state = self.BALL_IS_OUTSIDE
            return

        # 自チームディフェンスエリア侵入判定
        if self._check_is_ball_in_defense_area(ball.pos, our_area=True):
            self._ball_state = self.BALL_IS_IN_OUR_DEFENSE_AREA
            return

        # 相手チームディフェンスエリア侵入判定
        if self._check_is_ball_in_defense_area(ball.pos, our_area=False):
            self._ball_state = self.BALL_IS_IN_THEIR_DEFENSE_AREA
            return

        # 自チームエリア侵入判定
        if self._check_is_ball_in_our_side(ball.pos):
            self._ball_state = self.BALL_IS_IN_OUR_SIDE
            return

        # 条件に入らなければ、相手チームエリアに侵入したと判定
        self._ball_state = self.BALL_IS_IN_THEIR_SIDE

    def _check_is_ball_outside(self, ball_pos):
        # ボールがフィールド外に出たか判定
        threshold_x = self._field_half_x
        threshold_y = self._field_half_y
        if self.ball_is_outside():
            threshold_x -= self.THRESHOLD_MARGIN
            threshold_y -= self.THRESHOLD_MARGIN

        if math.fabs(ball_pos.x) > threshold_x or math.fabs(ball_pos.y) > threshold_y:
            return True
        return False

    def _check_is_ball_in_defense_area(self, ball_pos, our_area=True):
        # ボールがディフェンスエリアに入ったか判定
        threshold_x = self._field_half_x - self._field_defense_x
        threshold_y = self._field_defense_half_y
        if self.ball_is_in_our_defense_area() or self.ball_is_in_their_defense_area():
            threshold_x -= self.THRESHOLD_MARGIN
            threshold_y += self.THRESHOLD_MARGIN

        if our_area and ball_pos.x < -threshold_x and math.fabs(ball_pos.y) < threshold_y:
            return True
        elif not our_area and ball_pos.x > threshold_x and math.fabs(ball_pos.y) < threshold_y:
            return True
        return False

    def _check_is_ball_in_our_side(self, ball_pos):
        # ボールがディフェンスエリアに入ったか判定
        threshold_x = 0.0
        if self.ball_is_in_our_side():
            threshold_x += self.THRESHOLD_MARGIN

        if ball_pos.x < threshold_x:
            return True
        return False

    def _update_ball_moving_state(self, ball):
        BALL_MOVING_THRESHOLD = 1.0  # m/s
        BALL_MOVING_HYSTERESIS = 0.3  # m/s

        # ボールが動いているか判定する
        if len(ball.vel) == 0:
            self._ball_is_moving = False
            return
        velocity_norm = math.hypot(ball.vel[0].x, ball.vel[0].y)

        # ボール速度がしきい値付近で揺れても、判定が切り替わらないようにヒステリシスを設ける
        threshold = BALL_MOVING_THRESHOLD
        if self._ball_is_moving:
            threshold = BALL_MOVING_THRESHOLD - BALL_MOVING_HYSTERESIS

        if velocity_norm > threshold:
            self._ball_is_moving = True
        else:
            self._ball_is_moving = False

    def get_ball_state(self):
        return self._ball_state

    def ball_is_outside(self):
        return self._ball_state == self.BALL_IS_OUTSIDE

    def ball_is_in_our_defense_area(self):
        return self._ball_state == self.BALL_IS_IN_OUR_DEFENSE_AREA

    def ball_is_in_their_defense_area(self):
        return self._ball_state == self.BALL_IS_IN_THEIR_DEFENSE_AREA

    def ball_is_in_our_side(self):
        return self._ball_state == self.BALL_IS_IN_OUR_SIDE
    
    def ball_is_moving(self):
        return self._ball_is_moving

    def _update_ball_placement_state(self, placement_position):
        ARRIVED_THRESHOLD = 0.13
        NEAR_THRESHOLD = 1.0
        THRESHOLD_MARGIN = 0.02
        diff_x = placement_position.x - self._ball.pos.x
        diff_y = placement_position.y - self._ball.pos.y
        distance = math.hypot(diff_x, diff_y)

        arrived_threshold = ARRIVED_THRESHOLD
        near_threshold = NEAR_THRESHOLD
        if self._ball_placement_state == self.BALL_PLACEMENT_ARRIVED_AT_TARGET:
            arrived_threshold += THRESHOLD_MARGIN
        elif self._ball_placement_state == self.BALL_PLACEMENT_NEAR_TARGET:
            near_threshold += THRESHOLD_MARGIN
        
        if distance < arrived_threshold:
            self._ball_placement_state = self.BALL_PLACEMENT_ARRIVED_AT_TARGET
        elif distance < near_threshold:
            self._ball_placement_state = self.BALL_PLACEMENT_NEAR_TARGET
        else:
            self._ball_placement_state = self.BALL_PLACEMENT_FAR_FROM_TARGET

    def get_ball_placement_state(self, placement_position):
        self._update_ball_placement_state(placement_position)
        return self._ball_placement_state
