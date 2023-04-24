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

class SubAttackerDecision(DecisionBase):

    def __init__(self, robot_operator, field_observer):
        super().__init__(robot_operator, field_observer)

        self._ZONE_TOPS = [FieldObserver.BALL_ZONE_LEFT_TOP, FieldObserver.BALL_ZONE_RIGHT_TOP,
                           FieldObserver.BALL_ZONE_LEFT_MID_TOP, FieldObserver.BALL_ZONE_RIGHT_MID_TOP]

        self._OUR_ZONE_TOPS = [FieldObserver.BALL_ZONE_LEFT_TOP, FieldObserver.BALL_ZONE_LEFT_MID_TOP]
        self._OUR_ZONE_BOTTOMS = [FieldObserver.BALL_ZONE_LEFT_BOTTOM, FieldObserver.BALL_ZONE_LEFT_MID_BOTTOM]

    def _offend(self, robot_id, base_id):
        # ボールがフィールド上半分にあるときは、フィールド下側に移動する
        if self._ball_zone_state in self._ZONE_TOPS:
            if self._act_id != base_id + 0:
                self._operator.move_to_ball_x(robot_id, -2.5)
                self._act_id = base_id + 0
            return
        else:
            if self._act_id != base_id + 1:
                self._operator.move_to_ball_x(robot_id, 2.5)
                self._act_id = base_id + 1
            return

    def _offend_with_kick(self, robot_id, base_id):
        # ボールがフィールド上半分にあるときは、フィールド下側に移動する
        if self._ball_zone_state in self._ZONE_TOPS:
            if self._act_id != base_id + 0:
                self._operator.move_to_ball_x_with_reflect(robot_id, -2.5)
                self._act_id = base_id + 0
            return
        else:
            if self._act_id != base_id + 1:
                self._operator.move_to_ball_x_with_reflect(robot_id, 2.5)
                self._act_id = base_id + 1
            return

    def _offend_our_side(self, robot_id, base_id):
        # ボールがフィールド上半分にあるときは、フィールド下側に移動する
        if self._ball_zone_state in self._OUR_ZONE_TOPS:
            if self._act_id != base_id + 0:
                self._operator.move_to_ball_x(robot_id, -2.5, -0.3)
                self._act_id = base_id + 0
            return
        elif self._ball_zone_state in self._OUR_ZONE_BOTTOMS:
            if self._act_id != base_id + 1:
                self._operator.move_to_ball_x(robot_id, 2.5, -0.3)
                self._act_id = base_id + 1
            return
        else:
            if self._act_id != base_id + 2:
                self._operator.move_to_ball_x(robot_id, -2.5, -0.3)
                self._act_id = base_id + 2
            return

    def stop(self, robot_id):
        self._offend(robot_id, self.ACT_ID_STOP)

    def inplay(self, robot_id):
        # self._offend(robot_id, self.ACT_ID_STOP)
        self._offend_with_kick(robot_id, self.ACT_ID_INPLAY)

    def our_pre_kickoff(self, robot_id):
        self._offend_our_side(robot_id, self.ACT_ID_PRE_KICKOFF)

    def our_kickoff(self, robot_id):
        self._offend_our_side(robot_id, self.ACT_ID_KICKOFF)

    def their_pre_kickoff(self, robot_id):
        self._offend_our_side(robot_id, self.ACT_ID_PRE_KICKOFF)

    def their_kickoff(self, robot_id):
        self._offend_our_side(robot_id, self.ACT_ID_KICKOFF)

    def our_pre_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_PENALTY:
            self._operator.move_to_look_ball(robot_id, -6.0 + 0.5, 4.5 - 0.3 * 5.0)
            self._act_id = self.ACT_ID_PRE_PENALTY

    def our_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_PENALTY:
            self._operator.move_to_look_ball(robot_id, -6.0 + 0.5, 4.5 - 0.3 * 5.0)
            self._act_id = self.ACT_ID_PRE_PENALTY

    def their_pre_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_PENALTY:
            self._operator.move_to_look_ball(robot_id, 6.0 - 0.5, 4.5 - 0.3 * 5.0)
            self._act_id = self.ACT_ID_PRE_PENALTY

    def their_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_PENALTY:
            self._operator.move_to_look_ball(robot_id, 6.0 - 0.5, 4.5 - 0.3 * 5.0)
            self._act_id = self.ACT_ID_PRE_PENALTY

    def our_penalty_inplay(self, robot_id):
        if self._act_id != self.ACT_ID_INPLAY:
            self._operator.stop(robot_id)
            self._act_id = self.ACT_ID_INPLAY

    def their_penalty_inplay(self, robot_id):
        if self._act_id != self.ACT_ID_INPLAY:
            self._operator.stop(robot_id)
            self._act_id = self.ACT_ID_INPLAY

    def our_direct(self, robot_id):
        self._offend(robot_id, self.ACT_ID_DIRECT)

    def their_direct(self, robot_id):
        self._offend(robot_id, self.ACT_ID_DIRECT)

    def our_indirect(self, robot_id):
        self._offend(robot_id, self.ACT_ID_INDIRECT)

    def their_indirect(self, robot_id):
        self._offend(robot_id, self.ACT_ID_INDIRECT)

    def our_ball_placement(self, robot_id, placement_pos):
        ID_FAR_FROM = self.ACT_ID_OUR_PLACEMENT + 0
        ID_NEAR = self.ACT_ID_OUR_PLACEMENT + 1
        ID_ARRIVED = self.ACT_ID_OUR_PLACEMENT + 2

        # プレースメントを回避しない
        self._operator.disable_avoid_placement(robot_id)

        if self._ball_placement_state == FieldObserver.BALL_PLACEMENT_FAR_FROM_TARGET:
            # ボールを受け取る
            if self._act_id != ID_FAR_FROM:
                self._operator.receive_from(robot_id, placement_pos.x, placement_pos.y, 0.3)
                self._act_id = ID_FAR_FROM
            return
        
        if self._ball_placement_state == FieldObserver.BALL_PLACEMENT_NEAR_TARGET:
            # 目標位置に近づきボールを支える
            if self._act_id != ID_NEAR:
                self._operator.receive_from(robot_id, placement_pos.x, placement_pos.y, 0.1, dynamic_receive=False)
                self._act_id = ID_NEAR
            return

        if self._ball_placement_state == FieldObserver.BALL_PLACEMENT_ARRIVED_AT_TARGET:
            # ボール位置が配置目標位置に到着したらボールから離れる
            if self._act_id != ID_ARRIVED:
                self._operator.approach_to_ball(robot_id, 0.6)
                self._act_id = ID_ARRIVED

    def their_ball_placement(self, robot_id, placement_pos):
        if self._act_id != self.ACT_ID_THEIR_PLACEMENT:
            self._operator.move_to_look_ball(robot_id, -6.0 + 2.0, 1.8 - 0.3 * 3.0)
            self._act_id = self.ACT_ID_THEIR_PLACEMENT
