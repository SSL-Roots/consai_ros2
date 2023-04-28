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

from decisions.zone_defese_base import ZoneDefenseDecisionBase
from decisions.zone_defese_base import ZoneDefenseID

class Zone1Decision(ZoneDefenseDecisionBase):

    def __init__(self, robot_operator, field_observer):
        super().__init__(robot_operator, field_observer)

    def stop(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_STOP, ZoneDefenseID.ZONE1, without_mark=True)

    def inplay(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_INPLAY, ZoneDefenseID.ZONE1)

    def our_pre_kickoff(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_PRE_KICKOFF, ZoneDefenseID.ZONE1, without_mark=True)

    def our_kickoff(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_KICKOFF, ZoneDefenseID.ZONE1, without_mark=True)

    def their_pre_kickoff(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_PRE_KICKOFF, ZoneDefenseID.ZONE1, without_mark=True)

    def their_kickoff(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_KICKOFF, ZoneDefenseID.ZONE1, without_mark=True)

    def our_pre_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_PENALTY:
            self._operator.move_to_look_ball(robot_id, -6.0 + 0.5, 4.5 - 0.3 * 6.0)
            self._act_id = self.ACT_ID_PRE_PENALTY

    def our_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_PENALTY:
            self._operator.move_to_look_ball(robot_id, -6.0 + 0.5, 4.5 - 0.3 * 6.0)
            self._act_id = self.ACT_ID_PRE_PENALTY

    def their_pre_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_PENALTY:
            self._operator.move_to_look_ball(robot_id, 6.0 - 0.5, 4.5 - 0.3 * 6.0)
            self._act_id = self.ACT_ID_PRE_PENALTY

    def their_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_PENALTY:
            self._operator.move_to_look_ball(robot_id, 6.0 - 0.5, 4.5 - 0.3 * 6.0)
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
        self._zone_defense(robot_id, self.ACT_ID_DIRECT, ZoneDefenseID.ZONE1, without_mark=False)

    def their_direct(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_DIRECT, ZoneDefenseID.ZONE1, without_mark=False)

    def our_indirect(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_INDIRECT, ZoneDefenseID.ZONE1)

    def their_indirect(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_INDIRECT, ZoneDefenseID.ZONE1)

    def our_ball_placement(self, robot_id, placement_pos):
        if self._act_id != self.ACT_ID_OUR_PLACEMENT:
            self._operator.move_to_look_ball(robot_id, -6.0 + 2.0, 1.8 - 0.3 * 4.0)
            self._act_id = self.ACT_ID_OUR_PLACEMENT

    def their_ball_placement(self, robot_id, placement_pos):
        if self._act_id != self.ACT_ID_THEIR_PLACEMENT:
            self._operator.move_to_look_ball(robot_id, -6.0 + 2.0, 1.8 - 0.3 * 4.0)
            self._act_id = self.ACT_ID_THEIR_PLACEMENT