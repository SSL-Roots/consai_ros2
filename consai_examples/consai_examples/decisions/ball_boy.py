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

from decisions.decision_base import DecisionBase
import math
from operation import Operation
from operation import TargetXY
from operation import TargetTheta


class BallBoyDecision(DecisionBase):

    def __init__(self, robot_operator, field_observer, invert=False):
        super().__init__(robot_operator, field_observer)
        # サイドチェンジに関わらず退避位置を常に同じするためにinvertフラグを取得する
        stand_by_x = 0.0
        stand_by_y = 4.5 + 0.15
        self._standby_theta = math.radians(90.0)

        if invert:
            stand_by_y *= -1.0
            self._standby_theta = math.radians(-90.0)

        # ボールボーイの待機位置
        self._standby_position = TargetXY.value(stand_by_x, stand_by_y) 

    def _standby_operation(self):
        return Operation().move_to_pose(
            self._standby_position, TargetTheta.value(self._standby_theta))

    def our_ball_placement(self, robot_id, placement_pos):
        move_to_ball = Operation().move_to_pose(
            self._standby_position, TargetTheta.value(self._standby_theta))
        move_to_ball = move_to_ball.disable_avoid_defense_area()
        dribble_operation = move_to_ball.with_ball_boy_dribbling_to(
            TargetXY.value(placement_pos.x, placement_pos.y))
        self._operator.operate(robot_id, dribble_operation)

    def their_ball_placement(self, robot_id, placement_pos):
        move_to_ball = Operation().move_to_pose(
            self._standby_position, TargetTheta.value(self._standby_theta))
        move_to_ball = move_to_ball.disable_avoid_defense_area()
        dribble_operation = move_to_ball.with_ball_boy_dribbling_to(
            TargetXY.value(placement_pos.x, placement_pos.y))
        self._operator.operate(robot_id, dribble_operation)


def gen_standby_function():
    def function(self, robot_id, placement_pos=None):
        operation = self._standby_operation()
        self._operator.operate(robot_id, operation)
    return function


for name in ['stop', 'inplay',
             'our_pre_kickoff', 'our_kickoff', 'their_pre_kickoff', 'their_kickoff',
             'our_pre_penalty', 'our_penalty', 'their_pre_penalty', 'their_penalty',
             'our_penalty_inplay', 'their_penalty_inplay',
             'our_direct', 'their_direct', 'our_indirect', 'their_indirect',
             'our_timeout', 'their_timeout']:
    setattr(BallBoyDecision, name, gen_standby_function())
