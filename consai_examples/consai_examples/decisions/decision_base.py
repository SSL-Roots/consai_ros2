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


class DecisionBase(object):

    def __init__(self, robot_operator):
        self._operator = robot_operator

    def halt(self, robot_id):
        print("HALT:{}".format(robot_id))
        self._operator.stop(robot_id)

    def stop(self, robot_id):
        print("STOP:{}".format(robot_id))
        self._operator.stop(robot_id)

    def inplay(self, robot_id):
        print("INPLAY:{}".format(robot_id))
        self._operator.stop(robot_id)

    def our_pre_kickoff(self, robot_id):
        print("OUR PRE KICKOFF:{}".format(robot_id))
        self._operator.stop(robot_id)

    def our_kickoff(self, robot_id):
        print("OUR KICKOFF:{}".format(robot_id))
        self._operator.stop(robot_id)

    def their_pre_kickoff(self, robot_id):
        print("THEIR PRE KICKOFF:{}".format(robot_id))
        self._operator.stop(robot_id)

    def their_kickoff(self, robot_id):
        print("THEIR KICKOFF:{}".format(robot_id))
        self._operator.stop(robot_id)

    def our_pre_penalty(self, robot_id):
        print("OUR PRE PENALTY:{}".format(robot_id))
        self._operator.stop(robot_id)

    def our_penalty(self, robot_id):
        print("OUR PENALTY:{}".format(robot_id))
        self._operator.stop(robot_id)

    def their_pre_penalty(self, robot_id):
        print("THEIR PRE PENALTY:{}".format(robot_id))
        self._operator.stop(robot_id)

    def their_penalty(self, robot_id):
        print("THEIR penalty:{}".format(robot_id))
        self._operator.stop(robot_id)

    def our_direct(self, robot_id):
        print("OUR DIRECT FREE KICK:{}".format(robot_id))
        self._operator.stop(robot_id)

    def their_direct(self, robot_id):
        print("THEIR DIRECT FREE KICK:{}".format(robot_id))
        self._operator.stop(robot_id)

    def our_indirect(self, robot_id):
        print("OUR INDIRECT FREE KICK:{}".format(robot_id))
        self._operator.stop(robot_id)

    def their_indirect(self, robot_id):
        print("THEIR INDIRECT FREE KICK:{}".format(robot_id))
        self._operator.stop(robot_id)

    def our_timeout(self, robot_id):
        print("OUR TIMEOUT:{}".format(robot_id))
        self._operator.stop(robot_id)

    def their_timeout(self, robot_id):
        print("THEIR TIMEOUT:{}".format(robot_id))
        self._operator.stop(robot_id)

    def our_ball_placement(self, robot_id, placement_pos):
        print("OUR BALL PLACEMENT:{} to x:{}, y:{}".format(
            robot_id, placement_pos.x, placement_pos.y))
        self._operator.stop(robot_id)

    def their_ball_placement(self, robot_id, placement_pos):
        print("THEIR BALL PLACEMENT:{} to x:{}, y:{}".format(
            robot_id, placement_pos.x, placement_pos.y))
        self._operator.stop(robot_id)
