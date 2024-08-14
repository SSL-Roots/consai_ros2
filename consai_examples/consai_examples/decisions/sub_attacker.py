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

from consai_examples.decisions.decision_base import DecisionBase
from consai_examples.operation import Operation
from consai_examples.operation import TargetXY
from consai_examples.operation import TargetTheta
from enum import Enum


class SubAttackerID(Enum):
    SUBATTACK1 = 0
    SUBATTACK2 = 1


class SubAttackerDecision(DecisionBase):

    def __init__(self, robot_operator, field_observer, sub_attacker_id: SubAttackerID):
        super().__init__(robot_operator, field_observer)
        self._sub_attacker_id = sub_attacker_id
        self._penalty_side_x = 3.5
        self._penalty_side_y = 1.5
        self._penalty_front_x = 2.6
        self._penalty_front_y = 1.4

    def _offend_operation(self):
        ball_pos = self._field_observer.detection().ball().pos()
        # ボールがフィールド上半分にあるときは、フィールド下側に移動する
        if ball_pos.y > 0:
            if self._sub_attacker_id == SubAttackerID.SUBATTACK1:
                move_to_ball = Operation().move_to_pose(TargetXY.value(
                    self._penalty_side_x, -self._penalty_side_y), TargetTheta.look_ball())
            else:
                move_to_ball = Operation().move_to_pose(TargetXY.value(
                    self._penalty_front_x, self._penalty_front_y), TargetTheta.look_ball())
        else:
            if self._sub_attacker_id == SubAttackerID.SUBATTACK1:
                move_to_ball = Operation().move_to_pose(TargetXY.value(
                    self._penalty_front_x, -self._penalty_front_y), TargetTheta.look_ball())
            else:
                move_to_ball = Operation().move_to_pose(TargetXY.value(
                    self._penalty_side_x, self._penalty_side_y), TargetTheta.look_ball())
        return move_to_ball

    def _offend_our_side_operation(self):
        # ボールがフィールド上半分にあるときは、フィールド下側に移動する
        move_to_ball = Operation().move_to_pose(TargetXY.ball(), TargetTheta.look_ball())
        if self._sub_attacker_id == SubAttackerID.SUBATTACK1:
            if self._field_observer.zone().ball_is_in_left_bottom() or \
                    self._field_observer.zone().ball_is_in_left_mid_bottom():
                move_to_ball = move_to_ball.overwrite_pose_y(1.2)
            else:
                move_to_ball = move_to_ball.overwrite_pose_y(-1.2)
        else:
            if self._field_observer.zone().ball_is_in_left_bottom() or \
                    self._field_observer.zone().ball_is_in_left_mid_bottom():
                move_to_ball = move_to_ball.overwrite_pose_y(-1.2)
            else:
                move_to_ball = move_to_ball.overwrite_pose_y(1.2)
        move_to_ball = move_to_ball.offset_pose_x(-0.3)
        return move_to_ball

    def stop(self, robot_id):
        ball_pos = self._field_observer.detection().ball().pos()
        if ball_pos.x > 0.5:
            operation = self._offend_operation()
        else:
            operation = self._offend_our_side_operation()
        operation = operation.enable_avoid_ball()
        operation = operation.enable_avoid_pushing_robots()
        self._operator.operate(robot_id, operation)

    def inplay(self, robot_id):
        operation = self._offend_operation()
        # シュート可能なIDリストを取得
        shoot_pos_list = self._field_observer.pass_shoot().get_shoot_pos_list()
        # シュートできる場合はシュートする
        if len(shoot_pos_list) > 0:
            operation = operation.with_reflecting_to(
                TargetXY.value(shoot_pos_list[0].x, shoot_pos_list[0].y))
        self._operator.operate(robot_id, operation)

    def our_pre_kickoff(self, robot_id):
        operation = self._offend_our_side_operation()
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)

    def our_kickoff(self, robot_id):
        operation = self._offend_our_side_operation()
        self._operator.operate(robot_id, operation)

    def their_pre_kickoff(self, robot_id):
        operation = self._offend_our_side_operation()
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)

    def their_kickoff(self, robot_id):
        operation = self._offend_our_side_operation()
        self._operator.operate(robot_id, operation)

    def our_direct(self, robot_id):
        operation = self._offend_operation()
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)

    def their_direct(self, robot_id):
        operation = self._offend_operation()
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)

    def our_indirect(self, robot_id):
        operation = self._offend_operation()
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)

    def their_indirect(self, robot_id):
        operation = self._offend_operation()
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)

    def our_ball_placement(self, robot_id, placement_pos):
        if self._field_observer.ball_placement().is_far_from(placement_pos) or \
           not self._field_observer.ball_placement().is_arrived_at(placement_pos):
            if self._sub_attacker_id == SubAttackerID.SUBATTACK1:
                # ボールを受け取る
                move_to_behind_target = Operation().move_on_line(
                    TargetXY.value(placement_pos.x, placement_pos.y),
                    TargetXY.ball(),
                    -0.1,
                    TargetTheta.look_ball())
                move_to_behind_target = move_to_behind_target.with_ball_receiving()
                self._operator.operate(robot_id, move_to_behind_target)
                return

        if self._sub_attacker_id == SubAttackerID.SUBATTACK1:
            # ボール位置が配置目標位置に到着したらボールから離れる
            avoid_ball = Operation().move_on_line(
                TargetXY.ball(), TargetXY.our_robot(robot_id), 0.6, TargetTheta.look_ball())
            self._operator.operate(robot_id, avoid_ball)
        else:
            operation = self._offend_operation()
            operation = operation.enable_avoid_ball()
            operation = operation.enable_avoid_placement_area(placement_pos)
            operation = operation.enable_avoid_pushing_robots()
            self._operator.operate(robot_id, operation)

    def their_ball_placement(self, robot_id, placement_pos):
        operation = self._offend_operation()
        operation = operation.enable_avoid_ball()
        operation = operation.enable_avoid_placement_area(placement_pos)
        operation = operation.enable_avoid_pushing_robots()
        self._operator.operate(robot_id, operation)

    def _our_penalty_operation(self):
        return Operation().move_to_pose(
            TargetXY.value(-self._PENALTY_WAIT_X, 3.1 - 0.3 * 5.0),
            TargetTheta.look_ball())

    def _their_penalty_operation(self):
        return Operation().move_to_pose(
            TargetXY.value(self._PENALTY_WAIT_X, 3.1 - 0.3 * 5.0),
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


for name in ['our_pre_penalty', 'our_penalty', 'our_penalty_inplay']:
    setattr(SubAttackerDecision, name, gen_our_penalty_function())

for name in ['their_pre_penalty', 'their_penalty', 'their_penalty_inplay']:
    setattr(SubAttackerDecision, name, gen_their_penalty_function())
