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

class AttackerDecision(DecisionBase):

    def __init__(self, robot_operator):
        super().__init__(robot_operator)

    def stop(self, robot_id):
        ID_IN_DEFENSE = self.ACT_ID_STOP + 0
        ID_CHASE = self.ACT_ID_STOP + 1
        # ボールがディフェンスエリアにあるときは、別の場所に移動する
        if self._ball_state == FieldObserver.BALL_IS_IN_OUR_DEFENSE_AREA:
            if self._act_id != ID_IN_DEFENSE:
                self._operator.move_to(robot_id, 0.0, 0.0, 0.0, True, self.MAX_VELOCITY_AT_STOP_GAME)
                self._act_id = ID_IN_DEFENSE
            return

        # ボールを追いかける
        if self._act_id != ID_CHASE:
            self._operator.chase_ball(robot_id, -0.6, 0.0, 0.0, look_from=False, keep=True)
            self._act_id = ID_CHASE

    def our_ball_placement(self, robot_id, placement_pos):
        ID_FAR_FROM = self.ACT_ID_OUR_PLACEMENT + 0
        ID_NEAR = self.ACT_ID_OUR_PLACEMENT + 1
        ID_ARRIVED = self.ACT_ID_OUR_PLACEMENT + 2

        if self._ball_placement_state == FieldObserver.BALL_PLACEMENT_FAR_FROM_TARGET:
            # ボール位置が配置目標位置から離れているときはパスする
            if self._act_id != ID_FAR_FROM:
                self._operator.pass_to(robot_id, placement_pos.x, placement_pos.y)
                self._act_id = ID_FAR_FROM
            return
        
        if self._ball_placement_state == FieldObserver.BALL_PLACEMENT_NEAR_TARGET:
            # ボール位置が配置目標位置に近づいたときはドリブルする
            if self._act_id != ID_NEAR:
                self._operator.dribble_to(robot_id, placement_pos.x, placement_pos.y)
                self._act_id = ID_NEAR
            return

        if self._ball_placement_state == FieldObserver.BALL_PLACEMENT_ARRIVED_AT_TARGET:
            # ボール位置が配置目標位置に到着したらボールから離れる
            if self._act_id != ID_ARRIVED:
                self._operator.approach_to_ball(robot_id, 0.6)
                self._act_id = ID_ARRIVED
