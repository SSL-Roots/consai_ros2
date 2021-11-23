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

from field_observer import FieldObserver

class DecisionBase(object):
    MAX_VELOCITY_AT_STOP_GAME = 1.5  # m/s
    ACT_ID_HALT = 10
    ACT_ID_STOP = 20
    ACT_ID_INPLAY = 30
    ACT_ID_PRE_KICKOFF = 40
    ACT_ID_KICKOFF = 50
    ACT_ID_PRE_PENALTY = 60
    ACT_ID_PENALTY = 70
    ACT_ID_DIRECT = 80
    ACT_ID_INDIRECT = 90
    ACT_ID_TIMEOUT = 100
    ACT_ID_OUR_PLACEMENT = 110
    ACT_ID_THEIR_PLACEMENT = 120

    def __init__(self, robot_operator):
        self._operator = robot_operator
        self._ball_state = FieldObserver.BALL_NONE
        self._ball_placement_state = FieldObserver.BALL_PLACEMENT_NONE
        self._act_id = -1

    def set_ball_state(self, ball_state):
        self._ball_state = ball_state

    def set_ball_placement_state(self, ball_placement_state):
        self._ball_placement_state = ball_placement_state

    def halt(self, robot_id):
        if self._act_id != self.ACT_ID_HALT:
            self._operator.stop(robot_id)
            self._act_id = self.ACT_ID_HALT

    def stop(self, robot_id):
        if self._act_id != self.ACT_ID_STOP:
            print("STOP:{}".format(robot_id))
            self._operator.stop(robot_id)
            self._act_id = self.ACT_ID_STOP

    def inplay(self, robot_id):
        if self._act_id != self.ACT_ID_INPLAY:
            print("INPLAY:{}".format(robot_id))
            self._operator.stop(robot_id)
            self._act_id = self.ACT_ID_INPLAY

    def our_pre_kickoff(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_KICKOFF:
            print("OUR PRE KICKOFF:{}".format(robot_id))
            self._operator.stop(robot_id)
            self._act_id = self.ACT_ID_PRE_KICKOFF

    def our_kickoff(self, robot_id):
        if self._act_id != self.ACT_ID_KICKOFF:
            print("OUR KICKOFF:{}".format(robot_id))
            self._operator.stop(robot_id)
            self._act_id = self.ACT_ID_KICKOFF

    def their_pre_kickoff(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_KICKOFF:
            print("THEIR PRE KICKOFF:{}".format(robot_id))
            self._operator.stop(robot_id)
            self._act_id = self.ACT_ID_PRE_KICKOFF

    def their_kickoff(self, robot_id):
        if self._act_id != self.ACT_ID_KICKOFF:
            print("THEIR KICKOFF:{}".format(robot_id))
            self._operator.stop(robot_id)
            self._act_id = self.ACT_ID_KICKOFF

    def our_pre_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_PENALTY:
            print("OUR PRE PENALTY:{}".format(robot_id))
            self._operator.stop(robot_id)
            self._act_id = self.ACT_ID_PRE_PENALTY

    def our_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PENALTY:
            print("OUR PENALTY:{}".format(robot_id))
            self._operator.stop(robot_id)
            self._act_id = self.ACT_ID_PENALTY

    def their_pre_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_PENALTY:
            print("THEIR PRE PENALTY:{}".format(robot_id))
            self._operator.stop(robot_id)
            self._act_id = self.ACT_ID_PRE_PENALTY

    def their_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PENALTY:
            print("THEIR penalty:{}".format(robot_id))
            self._operator.stop(robot_id)
            self._act_id = self.ACT_ID_PENALTY

    def our_direct(self, robot_id):
        if self._act_id != self.ACT_ID_DIRECT:
            print("OUR DIRECT FREE KICK:{}".format(robot_id))
            self._operator.stop(robot_id)
            self._act_id = self.ACT_ID_DIRECT

    def their_direct(self, robot_id):
        if self._act_id != self.ACT_ID_DIRECT:
            print("THEIR DIRECT FREE KICK:{}".format(robot_id))
            self._operator.stop(robot_id)
            self._act_id = self.ACT_ID_DIRECT

    def our_indirect(self, robot_id):
        if self._act_id != self.ACT_ID_INDIRECT:
            print("OUR INDIRECT FREE KICK:{}".format(robot_id))
            self._operator.stop(robot_id)
            self._act_id = self.ACT_ID_INDIRECT

    def their_indirect(self, robot_id):
        if self._act_id != self.ACT_ID_INDIRECT:
            print("THEIR INDIRECT FREE KICK:{}".format(robot_id))
            self._operator.stop(robot_id)
            self._act_id = self.ACT_ID_INDIRECT

    def our_timeout(self, robot_id):
        if self._act_id != self.ACT_ID_TIMEOUT:
            print("OUR TIMEOUT:{}".format(robot_id))
            self._operator.stop(robot_id)
            self._act_id = self.ACT_ID_TIMEOUT

    def their_timeout(self, robot_id):
        if self._act_id != self.ACT_ID_TIMEOUT:
            print("THEIR TIMEOUT:{}".format(robot_id))
            self._operator.stop(robot_id)
            self._act_id = self.ACT_ID_TIMEOUT

    def our_ball_placement(self, robot_id, placement_pos):
        if self._act_id != self.ACT_ID_OUR_PLACEMENT:
            print("OUR BALL PLACEMENT:{} to x:{}, y:{}".format(
                robot_id, placement_pos.x, placement_pos.y))
            self._operator.stop(robot_id)
            self._act_id = self.ACT_ID_OUR_PLACEMENT

    def their_ball_placement(self, robot_id, placement_pos):
        if self._act_id != self.ACT_ID_THEIR_PLACEMENT:
            print("THEIR BALL PLACEMENT:{} to x:{}, y:{}".format(
                robot_id, placement_pos.x, placement_pos.y))
            self._operator.stop(robot_id)
            self._act_id = self.ACT_ID_THEIR_PLACEMENT
