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

class GoaleDecision(DecisionBase):

    def __init__(self, robot_operator):
        super().__init__(robot_operator)

    def _defend_goal(self, robot_id):
        p1_x = -6.0 + 0.3
        p1_y = 0.9
        p2_x = -6.0 + 0.3
        p2_y = -0.9
        self._operator.move_to_line_to_defend_our_goal(robot_id, p1_x, p1_y, p2_x, p2_y)
        # self._operator.move_to_line_to_defend_our_goal_with_reflect(robot_id, p1_x, p1_y, p2_x, p2_y)

    def _defend_goal_with_kick(self, robot_id):
        p1_x = -6.0 + 0.3
        p1_y = 0.9
        p2_x = -6.0 + 0.3
        p2_y = -0.9
        self._operator.move_to_line_to_defend_our_goal_with_reflect(robot_id, p1_x, p1_y, p2_x, p2_y)

    def stop(self, robot_id):
        if self._act_id != self.ACT_ID_STOP:
            self._defend_goal(robot_id)
            self._act_id = self.ACT_ID_STOP

    def inplay(self, robot_id):
        ID_IN_DEFENSE = self.ACT_ID_INPLAY + 0
        ID_IN_PLAY = self.ACT_ID_INPLAY + 1

        # ボールがディフェンスエリアにあるときは、ボールを蹴る
        if self._ball_state == FieldObserver.BALL_IS_IN_OUR_DEFENSE_AREA:
            if self._act_id != ID_IN_DEFENSE:
                self._operator.shoot_to(robot_id, 5.0, 0.0)
                self._act_id = ID_IN_DEFENSE
            return

        if self._act_id != ID_IN_PLAY:
            # self._defend_goal(robot_id)
            self._defend_goal_with_kick(robot_id)
            self._act_id = ID_IN_PLAY

    def our_pre_kickoff(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_KICKOFF:
            self._defend_goal(robot_id)
            self._act_id = self.ACT_ID_PRE_KICKOFF

    def our_kickoff(self, robot_id):
        if self._act_id != self.ACT_ID_KICKOFF:
            self._defend_goal(robot_id)
            self._act_id = self.ACT_ID_KICKOFF

    def their_pre_kickoff(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_KICKOFF:
            self._defend_goal(robot_id)
            self._act_id = self.ACT_ID_PRE_KICKOFF

    def their_kickoff(self, robot_id):
        if self._act_id != self.ACT_ID_KICKOFF:
            self._defend_goal(robot_id)
            self._act_id = self.ACT_ID_KICKOFF

    def our_pre_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_PENALTY:
            self._defend_goal(robot_id)
            self._act_id = self.ACT_ID_PRE_PENALTY

    def our_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PENALTY:
            self._defend_goal(robot_id)
            self._act_id = self.ACT_ID_PENALTY

    def their_pre_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_PENALTY:
            self._defend_goal(robot_id)
            self._act_id = self.ACT_ID_PRE_PENALTY

    def their_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PENALTY:
            self._defend_goal(robot_id)
            self._act_id = self.ACT_ID_PENALTY

    def our_direct(self, robot_id):
        if self._act_id != self.ACT_ID_DIRECT:
            self._defend_goal(robot_id)
            self._act_id = self.ACT_ID_DIRECT

    def their_direct(self, robot_id):
        if self._act_id != self.ACT_ID_DIRECT:
            self._defend_goal(robot_id)
            self._act_id = self.ACT_ID_DIRECT

    def our_indirect(self, robot_id):
        if self._act_id != self.ACT_ID_INDIRECT:
            self._defend_goal(robot_id)
            self._act_id = self.ACT_ID_INDIRECT

    def their_indirect(self, robot_id):
        if self._act_id != self.ACT_ID_INDIRECT:
            self._defend_goal(robot_id)
            self._act_id = self.ACT_ID_INDIRECT

    def our_ball_placement(self, robot_id, placement_pos):
        if self._act_id != self.ACT_ID_OUR_PLACEMENT:
            self._defend_goal(robot_id)
            self._act_id = self.ACT_ID_OUR_PLACEMENT

    def their_ball_placement(self, robot_id, placement_pos):
        if self._act_id != self.ACT_ID_THEIR_PLACEMENT:
            self._defend_goal(robot_id)
            self._act_id = self.ACT_ID_THEIR_PLACEMENT
