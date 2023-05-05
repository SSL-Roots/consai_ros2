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
from field_observer import FieldObserver

class CenterBackID(Enum):
    # ゾーンディフェンスは最大4台まで
    CENTER_BACK1 = 0
    CENTER_BACK2 = 1

class CenterBackDecision(DecisionBase):

    def __init__(self, robot_operator, field_observer, center_back_id: CenterBackID):
        super().__init__(robot_operator, field_observer)
        self._center_back_id = center_back_id

        self._our_penalty_pos_x = -self._PENALTY_WAIT_X
        self._our_penalty_pos_y = 4.5 - 0.3 * (1.0 + self._center_back_id.value)
        self._their_penalty_pos_x = self._PENALTY_WAIT_X
        self._their_penalty_pos_y = 4.5 - 0.3 * (1.0 + self._center_back_id.value)
        self._ball_placement_pos_x = -6.0 + 2.0
        self._ball_placement_pos_y = 1.8 - 0.3 * (1.0 + self._center_back_id.value)

    def _defend_upper_defense_area(self, robot_id, base_id):
        # ディフェンスエリアの上半分を守る
        ID_UPPER = base_id + 100
        ID_FRONT = base_id + 200
        ID_LOWER = base_id + 300

        # ボールが左上にあれば上側をまもる
        if self._ball_zone_state == FieldObserver.BALL_ZONE_LEFT_TOP:
            if self._act_id != ID_UPPER:
                self._defend_upper_top_defense_area(robot_id)
                self._act_id = ID_UPPER
        elif self._ball_zone_state == FieldObserver.BALL_ZONE_LEFT_BOTTOM:
            if self._act_id != ID_LOWER:
                self._defend_lower_bottom_defense_area(robot_id)
                self._act_id = ID_LOWER
        else:
            if self._act_id != ID_FRONT:
                self._defend_upper_front_defense_area(robot_id)
                self._act_id = ID_FRONT

    def _defend_upper_front_defense_area(self, robot_id):
        # ディフェンスエリアの前側を守る
        p1_x = -6.0 + 1.8 + 0.5
        p1_y = 1.8
        p2_x = -6.0 + 1.8 + 0.5
        p2_y = -1.8
        # self._operator.move_to_line_to_defend_our_goal(robot_id, p1_x, p1_y, p2_x, p2_y)
        self._operator.move_to_line_to_defend_our_goal_with_reflect(robot_id, p1_x, p1_y, p2_x, p2_y)

    def _defend_upper_front_defense_area_with_kick(self, robot_id):
        # ディフェンスエリアの前側を守る(リフレクトキック狙い)
        p1_x = -6.0 + 1.8 + 0.5
        p1_y = 1.8
        p2_x = -6.0 + 1.8 + 0.5
        p2_y = -1.8
        self._operator.move_to_line_to_defend_our_goal_with_reflect(robot_id, p1_x, p1_y, p2_x, p2_y)

    def _defend_lower_bottom_defense_area(self, robot_id):
        # ディフェンスエリアの下側(yマイナス側)を守る
        p1_x = -6.0 + 0.3
        p1_y = -1.8 - 0.5
        p2_x = -6.0 + 1.8 + 0.3
        p2_y = -1.8 - 0.5
        self._operator.move_to_line_to_defend_our_goal_with_reflect(robot_id, p1_x, p1_y, p2_x, p2_y)

    def _defend_upper_top_defense_area(self, robot_id):
        # ディフェンスエリアの上側(yプラス側)を守る
        p1_x = -6.0 + 0.3
        p1_y = 1.8 + 0.5
        p2_x = -6.0 + 1.8 + 0.3
        p2_y = 1.8 + 0.5
        # self._operator.move_to_line_to_defend_our_goal(robot_id, p1_x, p1_y, p2_x, p2_y)
        self._operator.move_to_line_to_defend_our_goal_with_reflect(robot_id, p1_x, p1_y, p2_x, p2_y)

    

    def stop(self, robot_id):
        if self._act_id != self.ACT_ID_STOP:
            self._defend_upper_front_defense_area(robot_id)
            self._act_id = self.ACT_ID_STOP

    def inplay(self, robot_id):
        self._defend_upper_defense_area(robot_id, self.ACT_ID_INPLAY)

    def our_pre_kickoff(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_KICKOFF:
            self._defend_upper_front_defense_area(robot_id)
            self._act_id = self.ACT_ID_PRE_KICKOFF

    def our_kickoff(self, robot_id):
        if self._act_id != self.ACT_ID_KICKOFF:
            self._defend_upper_front_defense_area(robot_id)
            self._act_id = self.ACT_ID_KICKOFF

    def their_pre_kickoff(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_KICKOFF:
            self._defend_upper_front_defense_area(robot_id)
            self._act_id = self.ACT_ID_PRE_KICKOFF

    def their_kickoff(self, robot_id):
        if self._act_id != self.ACT_ID_KICKOFF:
            self._defend_upper_front_defense_area(robot_id)
            self._act_id = self.ACT_ID_KICKOFF

    def our_pre_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_PENALTY:
            self._operator.move_to_look_ball(robot_id, self._our_penalty_pos_x, self._our_penalty_pos_y)
            self._act_id = self.ACT_ID_PRE_PENALTY

    def our_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PENALTY:
            self._operator.move_to_look_ball(robot_id, self._our_penalty_pos_x, self._our_penalty_pos_y)
            self._act_id = self.ACT_ID_PENALTY

    def their_pre_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_PENALTY:
            self._operator.move_to_look_ball(robot_id, self._their_penalty_pos_x, self._their_penalty_pos_y)
            self._act_id = self.ACT_ID_PRE_PENALTY

    def their_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PENALTY:
            self._operator.move_to_look_ball(robot_id, self._their_penalty_pos_x, self._their_penalty_pos_y)
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
            self._defend_upper_front_defense_area(robot_id)
            self._act_id = self.ACT_ID_DIRECT

    def their_direct(self, robot_id):
        if self._act_id != self.ACT_ID_DIRECT:
            self._defend_upper_front_defense_area(robot_id)
            self._act_id = self.ACT_ID_DIRECT

    def our_indirect(self, robot_id):
        if self._act_id != self.ACT_ID_INDIRECT:
            self._defend_upper_front_defense_area(robot_id)
            self._act_id = self.ACT_ID_INDIRECT

    def their_indirect(self, robot_id):
        if self._act_id != self.ACT_ID_INDIRECT:
            self._defend_upper_front_defense_area(robot_id)
            self._act_id = self.ACT_ID_INDIRECT

    def our_ball_placement(self, robot_id, placement_pos):
        if self._act_id != self.ACT_ID_OUR_PLACEMENT:
            self._operator.move_to_look_ball(robot_id, self._ball_placement_pos_x, self._ball_placement_pos_y)
            self._act_id = self.ACT_ID_OUR_PLACEMENT

    def their_ball_placement(self, robot_id, placement_pos):
        if self._act_id != self.ACT_ID_THEIR_PLACEMENT:
            self._operator.move_to_look_ball(robot_id, self._ball_placement_pos_x, self._ball_placement_pos_y)
            self._act_id = self.ACT_ID_THEIR_PLACEMENT
