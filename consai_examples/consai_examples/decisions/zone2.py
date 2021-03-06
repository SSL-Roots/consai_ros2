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

class Zone2Decision(DecisionBase):

    def __init__(self, robot_operator):
        super().__init__(robot_operator)

    def _zone_defense(self, robot_id, base_id, without_mark=False):
        # ゾーンディフェンスの担当者数に合わせて、待機位置を変更する
        ID_DEFEND_BALL = base_id + self._num_of_zone_roles
        ID_IN_ZONE = base_id + self._num_of_zone_roles + 100
        ID_MAN_MARK = base_id + self._num_of_zone_roles + 200
        ID_IN_OUR_DEFENSE = base_id + self._num_of_zone_roles + 300

        # ボールが自分ディフェンスエリアにあるときは、ボールと同じ軸上に移動する
        if self._ball_state == FieldObserver.BALL_IS_IN_OUR_DEFENSE_AREA:
            if self._act_id != ID_IN_OUR_DEFENSE:
                self._operator.move_to_ball_y(robot_id, -2.0)
                self._act_id = ID_IN_OUR_DEFENSE
            return

        # # ゾーン内にボールがあれば、ボールを追いかける

        ball_is_in_my_zone = False
        if self._ball_state == FieldObserver.BALL_IS_IN_OUR_SIDE:
            if self._num_of_zone_roles == 2:
                if self._ball_zone_state in [FieldObserver.BALL_ZONE_LEFT_MID_BOTTOM,
                                             FieldObserver.BALL_ZONE_LEFT_BOTTOM]:
                    ball_is_in_my_zone = True
            elif self._num_of_zone_roles == 3:
                if self._ball_zone_state in [FieldObserver.BALL_ZONE_LEFT_MID_TOP,
                                             FieldObserver.BALL_ZONE_LEFT_MID_BOTTOM]:
                    ball_is_in_my_zone = True
            else:
                if self._ball_zone_state in [FieldObserver.BALL_ZONE_LEFT_MID_TOP]:
                    ball_is_in_my_zone = True

            # if chase_ball:
            #     if self._act_id != ID_DEFEND_BALL:
            #         self._operator.move_to_defend_our_goal_from_ball(robot_id, 0.9)
            #         self._act_id = ID_DEFEND_BALL
            #     return

        # ゾーン内の相手ロボットがいれば、ボールとロボットの間に移動する
        if self._zone_targets[1] is not None and without_mark is False and ball_is_in_my_zone is False:
            if self._act_id != ID_MAN_MARK:
                self._operator.man_mark(robot_id, self._zone_targets[1], 0.5)
                self._act_id = ID_MAN_MARK 
            return


        # ゾーン内で待機する
        if self._act_id != ID_IN_ZONE:
            target_x = -2.0
            target_y = -4.5 * 0.5
            if self._num_of_zone_roles == 3:
                target_y = 0.0
            elif self._num_of_zone_roles == 4:
                target_y = 4.5 * 0.25
            # self._operator.move_to_receive(robot_id, target_x, target_y)
            self._operator.move_to_reflect_shoot_to_their_goal(robot_id, target_x, target_y)
            self._act_id = ID_IN_ZONE 
        return

    def stop(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_STOP, without_mark=True)

    def inplay(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_INPLAY)

    def our_pre_kickoff(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_PRE_KICKOFF, without_mark=True)

    def our_kickoff(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_KICKOFF, without_mark=True)

    def their_pre_kickoff(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_PRE_KICKOFF, without_mark=True)

    def their_kickoff(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_KICKOFF, without_mark=True)

    def our_pre_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_PENALTY:
            self._operator.move_to_look_ball(robot_id, -6.0 + 0.5, 4.5 - 0.3 * 7.0)
            self._act_id = self.ACT_ID_PRE_PENALTY

    def our_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_PENALTY:
            self._operator.move_to_look_ball(robot_id, -6.0 + 0.5, 4.5 - 0.3 * 7.0)
            self._act_id = self.ACT_ID_PRE_PENALTY

    def their_pre_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_PENALTY:
            self._operator.move_to_look_ball(robot_id, 6.0 - 0.5, 4.5 - 0.3 * 7.0)
            self._act_id = self.ACT_ID_PRE_PENALTY

    def their_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_PENALTY:
            self._operator.move_to_look_ball(robot_id, 6.0 - 0.5, 4.5 - 0.3 * 7.0)
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
        self._zone_defense(robot_id, self.ACT_ID_DIRECT, without_mark=False)

    def their_direct(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_DIRECT, without_mark=False)

    def our_indirect(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_INDIRECT)

    def their_indirect(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_INDIRECT)

    def our_ball_placement(self, robot_id, placement_pos):
        if self._act_id != self.ACT_ID_OUR_PLACEMENT:
            self._operator.move_to_look_ball(robot_id, -6.0 + 2.0, 1.8 - 0.3 * 5.0)
            self._act_id = self.ACT_ID_OUR_PLACEMENT

    def their_ball_placement(self, robot_id, placement_pos):
        if self._act_id != self.ACT_ID_THEIR_PLACEMENT:
            self._operator.move_to_look_ball(robot_id, -6.0 + 2.0, 1.8 - 0.3 * 5.0)
            self._act_id = self.ACT_ID_THEIR_PLACEMENT
