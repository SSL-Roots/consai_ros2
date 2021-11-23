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

    def stop(self, robot_id):
        self._operator.stop(robot_id)

    def our_ball_placement(self, robot_id, placement_pos):
        if self._ball_placement_state == FieldObserver.BALL_PLACEMENT_FAR_FROM_TARGET:
            # ボールを受け取る
            self._operator.receive_from(robot_id, placement_pos.x, placement_pos.y, 0.3)
        elif self._ball_placement_state == FieldObserver.BALL_PLACEMENT_NEAR_TARGET:
            # 目標位置に近づきボールを支える
            self._operator.receive_from(robot_id, placement_pos.x, placement_pos.y, 0.1)
        elif self._ball_placement_state == FieldObserver.BALL_PLACEMENT_ARRIVED_AT_TARGET:
            # ボール位置が配置目標位置に到着したらボールから離れる
            self._operator.receive_from(robot_id, placement_pos.x, placement_pos.y, 0.6)
        else:
            self._operator.stop(robot_id)

