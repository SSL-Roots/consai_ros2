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

    def _defend_upper_defense_area(self, robot_id, base_id):
        # ディフェンスエリアの上半分を守る
        ID_UPPER = base_id + 100
        ID_FRONT = base_id + 200

        # ボールが左上にあれば上側をまもる
        if self._ball_zone_state == FieldObserver.BALL_ZONE_LEFT_TOP:
            if self._act_id != ID_UPPER:
                operation = self._defend_upper_top_operation()
                operation = operation.with_ball_receiving()
                operation = operation.with_reflecting_to(TargetXY.their_goal())
                self._operator.operate(robot_id, operation)
                self._act_id = ID_UPPER
        else:
            if self._act_id != ID_FRONT:
                operation = self._defend_upper_front_operation()
                operation = operation.with_ball_receiving()
                operation = operation.with_reflecting_to(TargetXY.their_goal())
                self._operator.operate(robot_id, operation)
                self._act_id = ID_FRONT

    def stop(self, robot_id):
        if self._act_id != self.ACT_ID_STOP:
            # self._defend_upper_front_defense_area(robot_id)
            operation = self._defend_upper_front_operation()
            self._operator.operate(robot_id, operation)
            self._act_id = self.ACT_ID_STOP

    def inplay(self, robot_id):
        self._defend_upper_defense_area(robot_id, self.ACT_ID_INPLAY)

    def our_pre_kickoff(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_KICKOFF:
            operation = self._defend_upper_front_operation()
            self._operator.operate(robot_id, operation)
            self._act_id = self.ACT_ID_PRE_KICKOFF

    def our_kickoff(self, robot_id):
        if self._act_id != self.ACT_ID_KICKOFF:
            operation = self._defend_upper_front_operation()
            self._operator.operate(robot_id, operation)
            self._act_id = self.ACT_ID_KICKOFF

    def their_pre_kickoff(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_KICKOFF:
            operation = self._defend_upper_front_operation()
            self._operator.operate(robot_id, operation)
            self._act_id = self.ACT_ID_PRE_KICKOFF

    def their_kickoff(self, robot_id):
        if self._act_id != self.ACT_ID_KICKOFF:
            operation = self._defend_upper_front_operation()
            operation = operation.with_ball_receiving()
            self._operator.operate(robot_id, operation)
            self._act_id = self.ACT_ID_KICKOFF

    def _our_penalty_operation(self):
        return Operation().move_to_pose(
            TargetXY.value(-self._PENALTY_WAIT_X, 4.5 - 0.3 * 1.0),
            TargetTheta.look_ball())

    def _their_penalty_operation(self):
        return Operation().move_to_pose(
            TargetXY.value(self._PENALTY_WAIT_X, 4.5 - 0.3 * 1.0),
            TargetTheta.look_ball())

    def our_pre_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_PENALTY:
            operation = self._our_penalty_operation()
            self._operator.operate(robot_id, operation)
            self._act_id = self.ACT_ID_PRE_PENALTY

    def our_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PENALTY:
            operation = self._our_penalty_operation()
            self._operator.operate(robot_id, operation)
            self._act_id = self.ACT_ID_PENALTY

    def their_pre_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_PENALTY:
            operation = self._their_penalty_operation()
            self._operator.operate(robot_id, operation)
            self._act_id = self.ACT_ID_PRE_PENALTY

    def their_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PENALTY:
            operation = self._their_penalty_operation()
            self._operator.operate(robot_id, operation)
            self._act_id = self.ACT_ID_PENALTY

    def our_penalty_inplay(self, robot_id):
        if self._act_id != self.ACT_ID_INPLAY:
            operation = self._our_penalty_operation()
            self._operator.operate(robot_id, operation)
            self._act_id = self.ACT_ID_INPLAY

    def their_penalty_inplay(self, robot_id):
        if self._act_id != self.ACT_ID_INPLAY:
            operation = self._their_penalty_operation()
            self._operator.operate(robot_id, operation)
            self._act_id = self.ACT_ID_INPLAY

    def our_direct(self, robot_id):
        if self._act_id != self.ACT_ID_DIRECT:
            operation = self._defend_upper_front_operation()
            operation = operation.with_ball_receiving()
            self._operator.operate(robot_id, operation)
            self._act_id = self.ACT_ID_DIRECT

    def their_direct(self, robot_id):
        if self._act_id != self.ACT_ID_DIRECT:
            operation = self._defend_upper_front_operation()
            operation = operation.with_ball_receiving()
            self._operator.operate(robot_id, operation)
            self._act_id = self.ACT_ID_DIRECT

    def our_indirect(self, robot_id):
        if self._act_id != self.ACT_ID_INDIRECT:
            operation = self._defend_upper_front_operation()
            operation = operation.with_ball_receiving()
            self._operator.operate(robot_id, operation)
            self._act_id = self.ACT_ID_INDIRECT

    def their_indirect(self, robot_id):
        if self._act_id != self.ACT_ID_INDIRECT:
            operation = self._defend_upper_front_operation()
            operation = operation.with_ball_receiving()
            self._operator.operate(robot_id, operation)
            self._act_id = self.ACT_ID_INDIRECT

    def _ball_placement_operation(self):
        return Operation().move_to_pose(
            TargetXY.value(-6.0 + 2.0, 1.8 - 0.3 * 1.0),
            TargetTheta.look_ball())

    def our_ball_placement(self, robot_id, placement_pos):
        if self._act_id != self.ACT_ID_OUR_PLACEMENT:
            operation = self._ball_placement_operation()
            self._operator.operate(robot_id, operation)
            self._act_id = self.ACT_ID_OUR_PLACEMENT

    def their_ball_placement(self, robot_id, placement_pos):
        if self._act_id != self.ACT_ID_THEIR_PLACEMENT:
            operation = self._ball_placement_operation()
            self._operator.operate(robot_id, operation)
            self._act_id = self.ACT_ID_THEIR_PLACEMENT
