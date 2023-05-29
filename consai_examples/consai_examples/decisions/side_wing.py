#!/usr/bin/env python3
# coding: UTF-8

# Copyright 2023 Roots
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


from enum import Enum

from decisions.decision_base import DecisionBase


class WingID(Enum):
    LEFT = 0
    RIGHT = 1


class SideWingDecision(DecisionBase):

    def __init__(self, robot_operator, field_observer, wing_id: WingID):
        super().__init__(robot_operator, field_observer)
        self._wing_id = wing_id
        self._our_penalty_pos_x = -self._PENALTY_WAIT_X
        self._our_penalty_pos_y = 4.5 - 0.3 * (3.0 + self._wing_id.value)
        self._their_penalty_pos_x = self._PENALTY_WAIT_X
        self._their_penalty_pos_y = 4.5 - 0.3 * (3.0 + self._wing_id.value)
        self._ball_placement_pos_x = -6.0 + 2.0
        self._ball_placement_pos_y = 1.8 - 0.3 * (8.0 + self._wing_id.value)

    def _defend_upper_defense_area(self, robot_id):
        # ディフェンスエリアの外側を守る
        p1_x = -6.0 + 0.3
        p1_y = (1.8 + 0.5) * (-1 if self._wing_id.value > 0 else 1)
        p2_x = -6.0 + 1.8 + 0.3
        p2_y = (1.8 + 0.5) * (-1 if self._wing_id.value > 0 else 1)
        self._operator.move_to_line_to_defend_our_goal(
            robot_id, p1_x, p1_y, p2_x, p2_y)

    def _defend_our_half_way(self, robot_id):
        # Half-wayラインの自陣側で待機
        target_x = -1.0
        target_y = 4.0 * (-1 if self._wing_id.value > 0 else 1)
        self._operator.move_to_reflect_shoot_to_their_goal(
                    robot_id, target_x, target_y)

    def _offend_upper_defense_area(self, robot_id):
        # 相手フィールドで待機する
        p1_x = 2.0
        p1_y = 4.0 * (-1 if self._wing_id.value > 0 else 1)
        p2_x = 5.0
        p2_y = 4.0 * (-1 if self._wing_id.value > 0 else 1)
        # self._operator.move_to_cross_line_our_center_and_ball_with_reflect(
        # robot_id, p1_x, p1_y, p2_x, p2_y)
        self._operator.move_to_cross_line_our_center_and_ball(
            robot_id, p1_x, p1_y, p2_x, p2_y)

    def stop(self, robot_id):
        if self._act_id != self.ACT_ID_STOP:
            # self._defend_our_half_way(robot_id)
            self._offend_upper_defense_area(robot_id)
            self._act_id = self.ACT_ID_STOP

    def inplay(self, robot_id):
        if self._act_id != self.ACT_ID_INPLAY:
            self._offend_upper_defense_area(robot_id)
            # self._defend_upper_defense_area(robot_id)
            self._act_id = self.ACT_ID_INPLAY

    def our_pre_kickoff(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_KICKOFF:
            self._defend_our_half_way(robot_id)
            self._act_id = self.ACT_ID_PRE_KICKOFF

    def our_kickoff(self, robot_id):
        if self._act_id != self.ACT_ID_KICKOFF:
            self._defend_our_half_way(robot_id)
            self._act_id = self.ACT_ID_KICKOFF

    def their_pre_kickoff(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_KICKOFF:
            self._defend_our_half_way(robot_id)
            self._act_id = self.ACT_ID_PRE_KICKOFF

    def their_kickoff(self, robot_id):
        if self._act_id != self.ACT_ID_KICKOFF:
            self._defend_our_half_way(robot_id)
            self._act_id = self.ACT_ID_KICKOFF

    def our_pre_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_PENALTY:
            self._operator.move_to_look_ball(
                robot_id, self._our_penalty_pos_x, self._our_penalty_pos_y)
            self._act_id = self.ACT_ID_PRE_PENALTY

    def our_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PENALTY:
            self._operator.move_to_look_ball(
                robot_id, self._our_penalty_pos_x, self._our_penalty_pos_y)
            self._act_id = self.ACT_ID_PENALTY

    def their_pre_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_PENALTY:
            self._operator.move_to_look_ball(
                robot_id, self._their_penalty_pos_x, self._their_penalty_pos_y)
            self._act_id = self.ACT_ID_PRE_PENALTY

    def their_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PENALTY:
            self._operator.move_to_look_ball(
                robot_id, self._their_penalty_pos_x, self._their_penalty_pos_y)
            self._act_id = self.ACT_ID_PENALTY

    def our_penalty_inplay(self, robot_id):
        if self._act_id != self.ACT_ID_INPLAY:
            self._operator.stop(robot_id)
            self._act_id = self.ACT_ID_INPLAY

    def their_penalty_inplay(self, robot_id):
        if self._act_id != self.ACT_ID_INPLAY:
            self._operator.stop(robot_id)
            self._act_id = self.ACT_ID_INPLAY

    def our_direct(self, robot_id):
        if self._act_id != self.ACT_ID_DIRECT:
            self._defend_our_half_way(robot_id)
            self._act_id = self.ACT_ID_DIRECT

    def their_direct(self, robot_id):
        if self._act_id != self.ACT_ID_DIRECT:
            self._defend_our_half_way(robot_id)
            self._act_id = self.ACT_ID_DIRECT

    def our_indirect(self, robot_id):
        if self._act_id != self.ACT_ID_INDIRECT:
            self._defend_our_half_way(robot_id)
            self._act_id = self.ACT_ID_INDIRECT

    def their_indirect(self, robot_id):
        if self._act_id != self.ACT_ID_INDIRECT:
            self._defend_our_half_way(robot_id)
            self._act_id = self.ACT_ID_INDIRECT

    def our_ball_placement(self, robot_id, placement_pos):
        if self._act_id != self.ACT_ID_OUR_PLACEMENT:
            self._operator.move_to_look_ball(
                robot_id, self._ball_placement_pos_x, self._ball_placement_pos_y)
            self._act_id = self.ACT_ID_OUR_PLACEMENT

    def their_ball_placement(self, robot_id, placement_pos):
        if self._act_id != self.ACT_ID_THEIR_PLACEMENT:
            self._operator.move_to_look_ball(
                robot_id, self._ball_placement_pos_x, self._ball_placement_pos_y)
            self._act_id = self.ACT_ID_THEIR_PLACEMENT
