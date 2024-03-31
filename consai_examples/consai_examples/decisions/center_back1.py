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
from field_observer import FieldObserver
from operation import Operation
from operation import TargetXY
from operation import TargetTheta


class CenterBack1Decision(DecisionBase):

    def __init__(self, robot_operator, field_observer):
        super().__init__(robot_operator, field_observer)

    def _defend_upper_front_operation(self):
        # ディフェンスエリアの前側上半分を守る
        p1_x = -6.0 + 1.8 + 0.5
        p1_y = 1.8
        p2_x = -6.0 + 1.8 + 0.5
        p2_y = 0.1
        return Operation().move_to_intersection(
            TargetXY.value(p1_x, p1_y), TargetXY.value(p2_x, p2_y),
            TargetXY.our_goal(), TargetXY.ball(), TargetTheta.look_ball())

    def _defend_upper_top_operation(self):
        # ディフェンスエリアの上半分を守る
        p1_x = -6.0 + 0.3
        p1_y = 1.8 + 0.5
        p2_x = -6.0 + 1.8 + 0.3
        p2_y = 1.8 + 0.5
        return Operation().move_to_intersection(
            TargetXY.value(p1_x, p1_y), TargetXY.value(p2_x, p2_y),
            TargetXY.our_goal(), TargetXY.ball(), TargetTheta.look_ball())

    def _defend_upper_defense_area(self, robot_id):
        # ディフェンスエリアの上半分を守る
        # ボールが左上にあれば上側をまもる
        if self._ball_zone_state == FieldObserver.BALL_ZONE_LEFT_TOP:
            operation = self._defend_upper_top_operation()
        else:
            operation = self._defend_upper_front_operation()
        operation = operation.with_ball_receiving()
        operation = operation.with_reflecting_to(TargetXY.their_goal())
        self._operator.operate(robot_id, operation)

    def stop(self, robot_id):
        operation = self._defend_upper_front_operation()
        operation = operation.enable_avoid_ball()
        operation = operation.enable_avoid_pushing_robots()
        self._operator.operate(robot_id, operation)

    def inplay(self, robot_id):
        self._defend_upper_defense_area(robot_id)

    def _our_penalty_operation(self):
        return Operation().move_to_pose(
            TargetXY.value(-self._PENALTY_WAIT_X, 4.5 - 0.3 * 1.0),
            TargetTheta.look_ball())

    def _their_penalty_operation(self):
        return Operation().move_to_pose(
            TargetXY.value(self._PENALTY_WAIT_X, 4.5 - 0.3 * 1.0),
            TargetTheta.look_ball())

    def _ball_placement_operation(self):
        return Operation().move_to_pose(
            TargetXY.value(-6.0 + 2.0, 1.8 - 0.3 * 1.0),
            TargetTheta.look_ball())


def gen_kickoff_function():
    def function(self, robot_id):
        operation = self._defend_upper_front_operation()
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)
    return function


def gen_setplay_function():
    def function(self, robot_id):
        operation = self._defend_upper_front_operation()
        operation = operation.with_ball_receiving()
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)
    return function


def gen_our_penalty_function():
    def function(self, robot_id):
        operation = self._our_penalty_operation()
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)
    return function


def gen_their_penalty_function():
    def function(self, robot_id):
        operation = self._their_penalty_operation()
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)
    return function


def gen_ball_placement_function():
    def function(self, robot_id, placement_pos):
        operation = self._ball_placement_operation()
        operation = operation.enable_avoid_placement_area(placement_pos)
        operation = operation.enable_avoid_pushing_robots()
        self._operator.operate(robot_id, operation)
    return function


for name in ['our_pre_kickoff', 'our_kickoff', 'their_pre_kickoff']:
    setattr(CenterBack1Decision, name, gen_kickoff_function())

for name in ['their_kickoff', 'our_direct', 'their_direct', 'our_indirect', 'their_indirect']:
    setattr(CenterBack1Decision, name, gen_setplay_function())

for name in ['our_pre_penalty', 'our_penalty', 'our_penalty_inplay']:
    setattr(CenterBack1Decision, name, gen_our_penalty_function())

for name in ['their_pre_penalty', 'their_penalty', 'their_penalty_inplay']:
    setattr(CenterBack1Decision, name, gen_their_penalty_function())

for name in ['our_ball_placement', 'their_ball_placement']:
    setattr(CenterBack1Decision, name, gen_ball_placement_function())
