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

class AttackerDecition(DecisionBase):

    def __init__(self, robot_operator):
        super().__init__(robot_operator)

    def stop(self, robot_id):
        # ボールがディフェンスエリアにあるときは、別の場所に移動する
        if self._ball_state == FieldObserver.BALL_IS_IN_OUR_DEFENSE_AREA:
            self._operator.move_to(robot_id, 0.0, 0.0, 0.0, True, self.MAX_VELOCITY_AT_STOP_GAME)
            return

        # ボールを追いかける
        self._operator.chase_ball(robot_id, -0.6, 0.0, 0.0, look_from=False, keep=True)
        print("ball_state:{}".format(self._ball_state))

    def our_ball_placement(self, robot_id, placement_pos):
        if self._ball_placement_state == FieldObserver.BALL_PLACEMENT_FAR_FROM_TARGET:
            # ボール位置が配置目標位置から離れているときはパスする
            self._operator.pass_to(robot_id, placement_pos.x, placement_pos.y)
        elif self._ball_placement_state == FieldObserver.BALL_PLACEMENT_NEAR_TARGET:
            # ボール位置が配置目標位置に近づいたときはドリブルする
            self._operator.dribble_to(robot_id, placement_pos.x, placement_pos.y)
        elif self._ball_placement_state == FieldObserver.BALL_PLACEMENT_ARRIVED_AT_TARGET:
            # ボール位置が配置目標位置に到着したらボールから離れる
            self._operator.move_to_defend_our_goal_from_ball(robot_id, 0.6)
        else:
            self._operator.stop(robot_id)

