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

class SubstituteDecision(DecisionBase):

    def __init__(self, robot_operator, field_observer, invert=False):
        super().__init__(robot_operator, field_observer)

        # サイドチェンジに関わらず退避位置を常に同じするためにinvertフラグを取得する
        self._invert = invert

    def _move_to_substitute_area(self, robot_id):
        # ロボット交代位置
        # ロボットは
        #    フィールドマージンに部分的接触している
        #    かつハーフウェーラインから1m離れない位置
        # で交代できる
        # ただしボールがハーフウェーラインから2m以上離れている場合
        # https://robocup-ssl.github.io/ssl-rules/sslrules.html#_robot_substitution
        target_name = "SUBSTITUTION_POS"
        pos_x = 0.0
        pos_y = -4.5 + -0.15
        if self._invert:
            pos_y *= -1.0
        self._operator.set_named_target(target_name, pos_x, pos_y)
        self._operator.publish_named_targets()
        self._operator.move_to_named_target(robot_id, target_name, keep=True)

    def stop(self, robot_id):
        if self._act_id != self.ACT_ID_STOP:
            self._move_to_substitute_area(robot_id)
            self._act_id = self.ACT_ID_STOP

    def inplay(self, robot_id):
        if self._act_id != self.ACT_ID_INPLAY:
            self._move_to_substitute_area(robot_id)
            self._act_id = self.ACT_ID_INPLAY

    def our_pre_kickoff(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_KICKOFF:
            self._move_to_substitute_area(robot_id)
            self._act_id = self.ACT_ID_PRE_KICKOFF

    def our_kickoff(self, robot_id):
        if self._act_id != self.ACT_ID_KICKOFF:
            self._move_to_substitute_area(robot_id)
            self._act_id = self.ACT_ID_KICKOFF

    def their_pre_kickoff(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_KICKOFF:
            self._move_to_substitute_area(robot_id)
            self._act_id = self.ACT_ID_PRE_KICKOFF

    def their_kickoff(self, robot_id):
        if self._act_id != self.ACT_ID_KICKOFF:
            self._move_to_substitute_area(robot_id)
            self._act_id = self.ACT_ID_KICKOFF

    def our_pre_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_PENALTY:
            self._move_to_substitute_area(robot_id)
            self._act_id = self.ACT_ID_PRE_PENALTY

    def our_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PENALTY:
            self._move_to_substitute_area(robot_id)
            self._act_id = self.ACT_ID_PENALTY

    def their_pre_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_PENALTY:
            self._move_to_substitute_area(robot_id)
            self._act_id = self.ACT_ID_PRE_PENALTY

    def their_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PENALTY:
            self._move_to_substitute_area(robot_id)
            self._act_id = self.ACT_ID_PENALTY

    def our_penalty_inplay(self, robot_id):
        if self._act_id != self.ACT_ID_INPLAY:
            self._move_to_substitute_area(robot_id)
            self._act_id = self.ACT_ID_INPLAY

    def their_penalty_inplay(self, robot_id):
        if self._act_id != self.ACT_ID_INPLAY:
            self._move_to_substitute_area(robot_id)
            self._act_id = self.ACT_ID_INPLAY

    def our_direct(self, robot_id):
        if self._act_id != self.ACT_ID_DIRECT:
            self._move_to_substitute_area(robot_id)
            self._act_id = self.ACT_ID_DIRECT

    def their_direct(self, robot_id):
        if self._act_id != self.ACT_ID_DIRECT:
            self._move_to_substitute_area(robot_id)
            self._act_id = self.ACT_ID_DIRECT

    def our_indirect(self, robot_id):
        if self._act_id != self.ACT_ID_INDIRECT:
            self._move_to_substitute_area(robot_id)
            self._act_id = self.ACT_ID_INDIRECT

    def their_indirect(self, robot_id):
        if self._act_id != self.ACT_ID_INDIRECT:
            self._move_to_substitute_area(robot_id)
            self._act_id = self.ACT_ID_INDIRECT

    def our_ball_placement(self, robot_id, placement_pos):
        if self._act_id != self.ACT_ID_OUR_PLACEMENT:
            self._move_to_substitute_area(robot_id)
            self._act_id = self.ACT_ID_OUR_PLACEMENT

    def their_ball_placement(self, robot_id, placement_pos):
        if self._act_id != self.ACT_ID_THEIR_PLACEMENT:
            self._move_to_substitute_area(robot_id)
            self._act_id = self.ACT_ID_THEIR_PLACEMENT
