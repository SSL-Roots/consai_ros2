#!/usr/bin/env python3
# coding: UTF-8

# Copyright 2023 Roots
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
from operation import Operation
from operation import TargetXY
from operation import TargetTheta


class SideBackID(Enum):
    SIDE1 = 0
    SIDE2 = 1


class SideBackDecision(DecisionBase):

    def __init__(self, robot_operator, field_observer, side_id: SideBackID):
        super().__init__(robot_operator, field_observer)
        self._side_id = side_id
        self._distance_from = 0.25
        self._our_penalty_pos_x = -self._PENALTY_WAIT_X
        self._our_penalty_pos_y = 4.5 - 0.3 * (3.0 + self._side_id.value)
        self._their_penalty_pos_x = self._PENALTY_WAIT_X
        self._their_penalty_pos_y = 4.5 - 0.3 * (3.0 + self._side_id.value)
        self._ball_placement_pos_x = -6.0 + 2.0
        self._ball_placement_pos_y = 1.8 - 0.3 * (8.0 + self._side_id.value)

    def _side_back_operation(self, without_mark=False):
        SIDE_ID = self._side_id.value

        # FIXME: ボールが相手側にある場合は攻めに役立ちそうなポジションに移動してほしい

        # DFエリア横に相手ロボットがいる、ボールとロボットの間に移動する
        if self._field_observer.side_back_target().has_target(SIDE_ID) and without_mark is False:
            target_id = self._field_observer.side_back_target().get_target_id(SIDE_ID)
            operation = Operation().move_on_line(
                TargetXY.their_robot(target_id), TargetXY.ball(),
                distance_from_p1=self._distance_from, target_theta=TargetTheta.look_ball())
            operation = operation.with_ball_receiving()
        else:
            # DFエリア横付近で待機する
            target_x, target_y = self._get_side_target_xy()
            operation = Operation().move_to_pose(
                TargetXY.value(target_x, target_y), TargetTheta.look_ball())
            operation = operation.with_reflecting_to(TargetXY.their_goal())
        return operation

    def _get_side_target_xy(self):
        # 待機場所をそれぞれの関数で算出する
        target_x = -3.5
        target_y = 0
        if self._side_id == SideBackID.SIDE1:
            return self._get_side1_target_xy()
        elif self._side_id == SideBackID.SIDE2:
            return self._get_side2_target_xy()
        return target_x, target_y

    def _get_side1_target_xy(self):
        target_x = -3.5
        target_y = 3.0
        return target_x, target_y

    def _get_side2_target_xy(self):
        target_x = -3.5
        target_y = -3.0
        return target_x, target_y

    def stop(self, robot_id):
        operation = self._side_back_operation(without_mark=True)
        operation = operation.enable_avoid_ball()
        operation = operation.enable_avoid_pushing_robots()
        self._operator.operate(robot_id, operation)

    def inplay(self, robot_id):
        operation = self._side_back_operation()
        self._operator.operate(robot_id, operation)

    def our_pre_kickoff(self, robot_id):
        operation = self._side_back_operation(without_mark=True)
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)

    def our_kickoff(self, robot_id):
        operation = self._side_back_operation(without_mark=True)
        self._operator.operate(robot_id, operation)

    def their_pre_kickoff(self, robot_id):
        operation = self._side_back_operation(without_mark=True)
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)

    def their_kickoff(self, robot_id):
        operation = self._side_back_operation(without_mark=True)
        self._operator.operate(robot_id, operation)

    def our_direct(self, robot_id):
        operation = self._side_back_operation()
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)

    def their_direct(self, robot_id):
        operation = self._side_back_operation()
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)

    def our_indirect(self, robot_id):
        operation = self._side_back_operation()
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)

    def their_indirect(self, robot_id):
        operation = self._side_back_operation()
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)

    def _our_penalty_operation(self):
        return Operation().move_to_pose(
            TargetXY.value(self._our_penalty_pos_x, self._our_penalty_pos_y),
            TargetTheta.look_ball())

    def _their_penalty_operation(self):
        return Operation().move_to_pose(
            TargetXY.value(self._their_penalty_pos_x, self._their_penalty_pos_y),
            TargetTheta.look_ball())

    def _ball_placement_operation(self):
        return Operation().move_to_pose(
            TargetXY.value(
                self._ball_placement_pos_x, self._ball_placement_pos_y),
            TargetTheta.look_ball())


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
    def function(self, robot_id, placement_pos=None):
        operation = self._ball_placement_operation()
        operation = operation.enable_avoid_placement_area(placement_pos)
        operation = operation.enable_avoid_pushing_robots()
        self._operator.operate(robot_id, operation)
    return function


for name in ['our_pre_penalty', 'our_penalty', 'our_penalty_inplay']:
    setattr(SideBackDecision, name, gen_our_penalty_function())

for name in ['their_pre_penalty', 'their_penalty', 'their_penalty_inplay']:
    setattr(SideBackDecision, name, gen_their_penalty_function())

for name in ['our_ball_placement', 'their_ball_placement']:
    setattr(SideBackDecision, name, gen_ball_placement_function())