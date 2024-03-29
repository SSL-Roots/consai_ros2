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
from operation import Operation
from operation import TargetXY
from operation import TargetTheta


class SideBack1Decision(DecisionBase):

    def __init__(self, robot_operator, field_observer):
        super().__init__(robot_operator, field_observer)

    def _defend_upper_defense_area(self, robot_id):
        # ディフェンスエリアの上側を守る
        p1_x = -6.0 + 0.3
        p1_y = 1.8 + 0.5
        p2_x = -6.0 + 1.8 + 0.3
        p2_y = 1.8 + 0.5
        operation = Operation().move_to_intersection(
            TargetXY.value(p1_x, p1_y), TargetXY.value(p2_x, p2_y),
            TargetXY.our_goal(), TargetXY.ball(), TargetTheta.look_ball())
        operation = operation.with_ball_receiving()
        self._operator.operate(robot_id, operation)

    def _offend_upper_defense_area(self, robot_id):
        # 相手フィールの上側を待機する
        p1_x = 2.0
        p1_y = 4.0
        p2_x = 3.0
        p2_y = 4.0
        operation = Operation().move_to_intersection(
            TargetXY.value(p1_x, p1_y), TargetXY.value(p2_x, p2_y),
            TargetXY.our_goal(), TargetXY.ball(), TargetTheta.look_ball())
        operation = operation.with_reflecting_to(TargetXY.their_goal())
        self._operator.operate(robot_id, operation)

    def stop(self, robot_id):
        self._defend_upper_defense_area(robot_id)

    def inplay(self, robot_id):
        self._offend_upper_defense_area(robot_id)

    def our_pre_kickoff(self, robot_id):
        self._defend_upper_defense_area(robot_id)

    def our_kickoff(self, robot_id):
        self._defend_upper_defense_area(robot_id)

    def their_pre_kickoff(self, robot_id):
        self._defend_upper_defense_area(robot_id)

    def their_kickoff(self, robot_id):
        self._defend_upper_defense_area(robot_id)

    def our_direct(self, robot_id):
        self._defend_upper_defense_area(robot_id)

    def their_direct(self, robot_id):
        self._defend_upper_defense_area(robot_id)

    def our_indirect(self, robot_id):
        self._defend_upper_defense_area(robot_id)

    def their_indirect(self, robot_id):
        self._defend_upper_defense_area(robot_id)

    def _our_penalty_operation(self):
        return Operation().move_to_pose(
            TargetXY.value(-self._PENALTY_WAIT_X, 4.5 - 0.3 * 3.0),
            TargetTheta.look_ball())

    def _their_penalty_operation(self):
        return Operation().move_to_pose(
            TargetXY.value(self._PENALTY_WAIT_X, 4.5 - 0.3 * 3.0),
            TargetTheta.look_ball())

    def _ball_placement_operation(self):
        return Operation().move_to_pose(
            TargetXY.value(-6.0 + 2.0, 1.8 - 0.3 * 8.0),
            TargetTheta.look_ball())


def gen_our_penalty_function():
    def function(self, robot_id):
        operation = self._our_penalty_operation()
        self._operator.operate(robot_id, operation)
    return function


def gen_their_penalty_function():
    def function(self, robot_id):
        operation = self._their_penalty_operation()
        self._operator.operate(robot_id, operation)
    return function


def gen_ball_placement_function():
    def function(self, robot_id, placement_pos=None):
        operation = self._ball_placement_operation()
        self._operator.operate(robot_id, operation)
    return function


for name in ['our_pre_penalty', 'our_penalty', 'our_penalty_inplay']:
    setattr(SideBack1Decision, name, gen_our_penalty_function())

for name in ['their_pre_penalty', 'their_penalty', 'their_penalty_inplay']:
    setattr(SideBack1Decision, name, gen_their_penalty_function())

for name in ['our_ball_placement', 'their_ball_placement']:
    setattr(SideBack1Decision, name, gen_ball_placement_function())
