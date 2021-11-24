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

    def __init__(self, robot_operator):
        super().__init__(robot_operator)

    def inplay(self, robot_id):
        if self._act_id != self.ACT_ID_INPLAY:
            self._operator.chase_ball(robot_id, -0.7, 0.1, 0.0, look_from=False, keep=True)
            self._act_id = self.ACT_ID_INPLAY

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
