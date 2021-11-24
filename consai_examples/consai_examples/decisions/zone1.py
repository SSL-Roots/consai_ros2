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

class Zone1Decision(DecisionBase):

    def __init__(self, robot_operator):
        super().__init__(robot_operator)

    def _zone_defense(self, robot_id, base_id):
        # ゾーンディフェンスの担当者数に合わせて、待機位置を変更する
        ID_DEFEND_BALL = base_id + self._num_of_zone_roles
        ID_IN_ZONE = base_id + self._num_of_zone_roles + 4

        # ゾーン内にボールがあれば、ボールを追いかける
        if self._ball_state == FieldObserver.BALL_IS_IN_OUR_SIDE:
            chase_ball = False
            if self._num_of_zone_roles == 1:
                chase_ball = True
            elif self._num_of_zone_roles == 2:
                if self._ball_zone_state in [FieldObserver.BALL_ZONE_LEFT_TOP,
                                             FieldObserver.BALL_ZONE_LEFT_MID_TOP]:
                    chase_ball = True
            elif self._num_of_zone_roles == 3 or self._num_of_zone_roles == 4:
                if self._ball_zone_state == FieldObserver.BALL_ZONE_LEFT_TOP:
                    chase_ball = True
            if chase_ball and self._act_id != ID_DEFEND_BALL:
                self._operator.move_to_defend_our_goal_from_ball(robot_id, 0.9)
                self._act_id = ID_DEFEND_BALL
            return

        # ゾーン内で待機する
        if self._act_id != ID_IN_ZONE:
            target_x = -2.0
            target_y = 0
            if self._num_of_zone_roles == 2:
                target_y = 4.5 * 0.5
            elif self._num_of_zone_roles >= 3:
                target_y = 4.5 * 0.75
            self._operator.move_to_receive(robot_id, target_x, target_y)
            self._act_id = ID_IN_ZONE 
        return

    def stop(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_STOP)

    def inplay(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_INPLAY)

    def our_pre_kickoff(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_PRE_KICKOFF)

    def our_kickoff(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_KICKOFF)

    def their_pre_kickoff(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_PRE_KICKOFF)

    def their_kickoff(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_KICKOFF)

    def our_pre_penalty(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_PRE_PENALTY)

    def our_penalty(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_PENALTY)

    def their_pre_penalty(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_PRE_PENALTY)

    def their_penalty(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_PENALTY)

    def our_direct(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_DIRECT)

    def their_direct(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_DIRECT)

    def our_indirect(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_INDIRECT)

    def their_indirect(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_INDIRECT)

    def our_ball_placement(self, robot_id, placement_pos):
        ID_FAR_FROM = self.ACT_ID_OUR_PLACEMENT + 0
        ID_NEAR = self.ACT_ID_OUR_PLACEMENT + 1
        ID_ARRIVED = self.ACT_ID_OUR_PLACEMENT + 2

        if self._ball_placement_state == FieldObserver.BALL_PLACEMENT_FAR_FROM_TARGET:
            # ボールを受け取る
            if self._act_id != ID_FAR_FROM:
                self._operator.receive_from(robot_id, placement_pos.x, placement_pos.y, 0.3)
                self._act_id = ID_FAR_FROM
            return
        
        if self._ball_placement_state == FieldObserver.BALL_PLACEMENT_NEAR_TARGET:
            # 目標位置に近づきボールを支える
            if self._act_id != ID_NEAR:
                self._operator.receive_from(robot_id, placement_pos.x, placement_pos.y, 0.2, dynamic_receive=False)
                self._act_id = ID_NEAR
            return

        if self._ball_placement_state == FieldObserver.BALL_PLACEMENT_ARRIVED_AT_TARGET:
            # ボール位置が配置目標位置に到着したらボールから離れる
            if self._act_id != ID_ARRIVED:
                self._operator.approach_to_ball(robot_id, 0.6)
                self._act_id = ID_ARRIVED
