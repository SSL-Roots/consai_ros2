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


class SubAttackerDecision(DecisionBase):

    def __init__(self, robot_operator, field_observer):
        super().__init__(robot_operator, field_observer)

        self._ZONE_TOPS = [FieldObserver.BALL_ZONE_LEFT_TOP,
                           FieldObserver.BALL_ZONE_RIGHT_TOP,
                           FieldObserver.BALL_ZONE_LEFT_MID_TOP,
                           FieldObserver.BALL_ZONE_RIGHT_MID_TOP]

        self._OUR_ZONE_TOPS = [FieldObserver.BALL_ZONE_LEFT_TOP,
                               FieldObserver.BALL_ZONE_LEFT_MID_TOP]
        self._OUR_ZONE_BOTTOMS = [FieldObserver.BALL_ZONE_LEFT_BOTTOM,
                                  FieldObserver.BALL_ZONE_LEFT_MID_BOTTOM]

    def _offend(self, robot_id):
        # ボールがフィールド上半分にあるときは、フィールド下側に移動する
        move_to_ball = Operation().move_to_pose(TargetXY.ball(), TargetTheta.look_ball())
        if self._ball_zone_state in self._ZONE_TOPS:
            move_to_ball = move_to_ball.overwrite_pose_y(-2.5)
        else:
            move_to_ball = move_to_ball.overwrite_pose_y(2.5)
        self._operator.operate(robot_id, move_to_ball)

    def _offend_our_side(self, robot_id):
        # ボールがフィールド上半分にあるときは、フィールド下側に移動する
        move_to_ball = Operation().move_to_pose(TargetXY.ball(), TargetTheta.look_ball())
        if self._ball_zone_state in self._OUR_ZONE_BOTTOMS:
            move_to_ball = move_to_ball.overwrite_pose_y(2.5)
        else:
            move_to_ball = move_to_ball.overwrite_pose_y(-2.5)

        move_to_ball = move_to_ball.offset_pose_x(-0.3)
        self._operator.operate(robot_id, move_to_ball)

    def stop(self, robot_id):
        self._offend(robot_id)

    def inplay(self, robot_id):
        self._offend(robot_id)

    def our_pre_kickoff(self, robot_id):
        self._offend_our_side(robot_id)

    def our_kickoff(self, robot_id):
        self._offend_our_side(robot_id)

    def their_pre_kickoff(self, robot_id):
        self._offend_our_side(robot_id)

    def their_kickoff(self, robot_id):
        self._offend_our_side(robot_id)

    def our_direct(self, robot_id):
        self._offend(robot_id)

    def their_direct(self, robot_id):
        self._offend(robot_id)

    def our_indirect(self, robot_id):
        self._offend(robot_id)

    def their_indirect(self, robot_id):
        self._offend(robot_id)

    def our_ball_placement(self, robot_id, placement_pos):
        if self._ball_placement_state == FieldObserver.BALL_PLACEMENT_FAR_FROM_TARGET or \
           self._ball_placement_state == FieldObserver.BALL_PLACEMENT_NEAR_TARGET:
            # ボールを受け取る
            move_to_behind_target = Operation().move_on_line(
                TargetXY.value(placement_pos.x, placement_pos.y),
                TargetXY.ball(),
                -0.1,
                TargetTheta.look_ball())
            move_to_behind_target = move_to_behind_target.with_ball_receiving()
            self._operator.operate(robot_id, move_to_behind_target)
            return

        if self._ball_placement_state == FieldObserver.BALL_PLACEMENT_ARRIVED_AT_TARGET:
            # ボール位置が配置目標位置に到着したらボールから離れる
            avoid_ball = Operation().move_on_line(
                TargetXY.ball(), TargetXY.our_robot(robot_id), 0.6, TargetTheta.look_ball())
            self._operator.operate(robot_id, avoid_ball)

    def their_ball_placement(self, robot_id, placement_pos):
        operation = Operation().move_to_pose(
            TargetXY.value(-6.0 + 2.0, 1.8 - 0.3 * 3.0),
            TargetTheta.look_ball())
        self._operator.operate(robot_id, operation)

    def _our_penalty_operation(self):
        return Operation().move_to_pose(
            TargetXY.value(-self._PENALTY_WAIT_X, 4.5 - 0.3 * 5.0),
            TargetTheta.look_ball())

    def _their_penalty_operation(self):
        return Operation().move_to_pose(
            TargetXY.value(self._PENALTY_WAIT_X, 4.5 - 0.3 * 5.0),
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


for name in ['our_pre_penalty', 'our_penalty', 'our_penalty_inplay']:
    setattr(SubAttackerDecision, name, gen_our_penalty_function())

for name in ['their_pre_penalty', 'their_penalty', 'their_penalty_inplay']:
    setattr(SubAttackerDecision, name, gen_their_penalty_function())
