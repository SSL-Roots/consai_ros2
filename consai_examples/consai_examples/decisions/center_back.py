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


from enum import Enum

from decisions.decision_base import DecisionBase
from field import Field
from operation import Operation
from operation import TargetXY
from operation import TargetTheta


class CenterBackID(Enum):
    # センターバックは最大2台まで
    CENTER_BACK1 = 0
    CENTER_BACK2 = 1


class CenterBackDecision(DecisionBase):

    def __init__(self, robot_operator, field_observer, center_back_id: CenterBackID):
        super().__init__(robot_operator, field_observer)
        self._center_back_id = center_back_id
        self._our_penalty_pos_y = 4.5 - 0.3 * (1.0 + self._center_back_id.value)
        self._their_penalty_pos_y = 4.5 - 0.3 * (1.0 + self._center_back_id.value)
        self._ball_placement_pos_x = -6.0 + 2.0
        self._ball_placement_pos_y = 1.8 - 0.3 * (1.0 + self._center_back_id.value)
        # ペナルティエリアからどれだけ離れるか
        self._MARGIN_LINE = 0.3
        # 2台でディフェンスする時のお互いの距離
        self._MARGIN_ROBOT = 0.1
        self._penalty_corner_upper_front = Field.penalty_pose('our', 'upper_front')
        self._penalty_corner_lower_front = Field.penalty_pose('our', 'lower_front')
        self._penalty_goalside_upper_back = Field.penalty_pose('our', 'upper_back')
        self._penalty_goalside_lower_back = Field.penalty_pose('our', 'lower_back')

    def _get_offset(self) -> float:
        offset = 0.0
        if self._num_of_center_back_roles > 1:
            if self._center_back_id == CenterBackID.CENTER_BACK1:
                offset = -self._MARGIN_ROBOT
            else:
                offset = self._MARGIN_ROBOT
        return offset

    def _defend_front_operation(self, offset: float):
        # ディフェンスエリアの前側を守る
        p1_x = self._penalty_corner_upper_front.x + self._MARGIN_LINE
        p1_y = self._penalty_corner_upper_front.y
        p2_x = self._penalty_corner_lower_front.x + self._MARGIN_LINE
        p2_y = self._penalty_corner_lower_front.y
        return Operation().move_to_intersection(
            TargetXY.value(p1_x, p1_y), TargetXY.value(p2_x, p2_y),
            TargetXY.our_goal(), TargetXY.ball(), TargetTheta.look_ball(), offset)

    def _defend_upper_operation(self, offset: float):
        # ディフェンスエリアの上側を守る
        p1_x = self._penalty_goalside_upper_back.x + 0.3
        p1_y = self._penalty_goalside_upper_back.y + self._MARGIN_LINE
        p2_x = self._penalty_corner_upper_front.x + 0.3
        p2_y = self._penalty_corner_upper_front.y + self._MARGIN_LINE
        return Operation().move_to_intersection(
            TargetXY.value(p1_x, p1_y), TargetXY.value(p2_x, p2_y),
            TargetXY.our_goal(), TargetXY.ball(), TargetTheta.look_ball(), offset)

    def _defend_lower_operation(self, offset: float):
        # ディフェンスエリアの下側を守る
        p1_x = self._penalty_corner_lower_front.x + 0.3
        p1_y = self._penalty_corner_lower_front.y - self._MARGIN_LINE
        p2_x = self._penalty_goalside_lower_back.x + 0.3
        p2_y = self._penalty_goalside_lower_back.y - self._MARGIN_LINE
        return Operation().move_to_intersection(
            TargetXY.value(p1_x, p1_y), TargetXY.value(p2_x, p2_y),
            TargetXY.our_goal(), TargetXY.ball(), TargetTheta.look_ball(), offset)

    def _defend_defense_area(self, robot_id):
        offset = self._get_offset()
        # ボール位置で行動を変更
        if self._field_observer.zone().ball_is_in_left_top():
            operation = self._defend_upper_operation(offset)
        elif self._field_observer.zone().ball_is_in_left_bottom():
            operation = self._defend_lower_operation(offset)
        else:
            operation = self._defend_front_operation(offset)
        operation = operation.with_ball_receiving()
        operation = operation.with_reflecting_to(TargetXY.their_goal())
        self._operator.operate(robot_id, operation)

    def stop(self, robot_id):
        offset = self._get_offset()
        operation = self._defend_front_operation(offset)
        operation = operation.enable_avoid_ball()
        operation = operation.enable_avoid_pushing_robots()
        self._operator.operate(robot_id, operation)

    def inplay(self, robot_id):
        self._defend_defense_area(robot_id)

    def _our_penalty_operation(self):
        return Operation().move_to_pose(
            TargetXY.value(-self._PENALTY_WAIT_X, self._our_penalty_pos_y),
            TargetTheta.look_ball())

    def _their_penalty_operation(self):
        return Operation().move_to_pose(
            TargetXY.value(self._PENALTY_WAIT_X, self._their_penalty_pos_y),
            TargetTheta.look_ball())

    def _ball_placement_operation(self):
        return Operation().move_to_pose(
            TargetXY.value(self._ball_placement_pos_x, self._ball_placement_pos_y),
            TargetTheta.look_ball())


def gen_kickoff_function():
    def function(self, robot_id):
        offset = self._get_offset()
        operation = self._defend_front_operation(offset)
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)
    return function


def gen_setplay_function():
    def function(self, robot_id):
        offset = self._get_offset()
        operation = self._defend_front_operation(offset)
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
        offset = self._get_offset()
        operation = self._defend_front_operation(offset)
        operation = operation.enable_avoid_ball()
        operation = operation.enable_avoid_placement_area(placement_pos)
        operation = operation.enable_avoid_pushing_robots()
        self._operator.operate(robot_id, operation)
    return function


for name in ['our_pre_kickoff', 'our_kickoff', 'their_pre_kickoff']:
    setattr(CenterBackDecision, name, gen_kickoff_function())

for name in ['their_kickoff', 'our_direct', 'their_direct', 'our_indirect', 'their_indirect']:
    setattr(CenterBackDecision, name, gen_setplay_function())

for name in ['our_pre_penalty', 'our_penalty', 'our_penalty_inplay']:
    setattr(CenterBackDecision, name, gen_our_penalty_function())

for name in ['their_pre_penalty', 'their_penalty', 'their_penalty_inplay']:
    setattr(CenterBackDecision, name, gen_their_penalty_function())

for name in ['our_ball_placement', 'their_ball_placement']:
    setattr(CenterBackDecision, name, gen_ball_placement_function())
