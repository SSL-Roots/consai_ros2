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
from operation import Operation


class DecisionBase(object):
    MAX_VELOCITY_AT_STOP_GAME = 1.5  # m/s
    ACT_ID_INIT = -1
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

    def __init__(self, robot_operator, field_observer):
        self._operator = robot_operator
        self._field_observer = field_observer
        self._ball_state = FieldObserver.BALL_NONE
        self._ball_placement_state = FieldObserver.BALL_PLACEMENT_NONE
        self._ball_zone_state = FieldObserver.BALL_ZONE_NONE
        self._num_of_zone_roles = 0
        self._zone_targets = {0: None, 1: None, 2: None, 3: None}
        self._act_id = self.ACT_ID_INIT
        self._PENALTY_WAIT_X = 4.1  # ペナルティキック待機位置のX座標

    def enable_stop_game_velocity(self, robot_id):
        self._operator.enable_stop_game_velocity(robot_id)

    def disable_stop_game_velocity(self, robot_id):
        self._operator.disable_stop_game_velocity(robot_id)

    def enable_avoid_placement(self, robot_id):
        self._operator.enable_avoid_placement(robot_id)

    def disable_avoid_placement(self, robot_id):
        self._operator.disable_avoid_placement(robot_id)

    def enable_avoid_obstacles(self, robot_id):
        self._operator.enable_avoid_obstacles(robot_id)

    def disable_avoid_obstacles(self, robot_id):
        self._operator.disable_avoid_obstacles(robot_id)

    def set_ball_state(self, ball_state):
        self._ball_state = ball_state

    def set_ball_placement_state(self, ball_placement_state):
        self._ball_placement_state = ball_placement_state

    def set_ball_zone_state(self, ball_zone_state):
        self._ball_zone_state = ball_zone_state

    def set_num_of_zone_roles(self, num_of_zone_roles):
        self._num_of_zone_roles = num_of_zone_roles

    def set_zone_targets(self, zone_targets):
        self._zone_targets = zone_targets

    def reset_act_id(self):
        self._act_id = self.ACT_ID_INIT

    def reset_operation(self, robot_id: int) -> None:
        self._operator.reset_operation(robot_id)


def generate_function():
    def function(self, robot_id):
        operation_halt = Operation().halt()
        self._operator.operate(robot_id, operation_halt)
    return function


function_names = [
    'halt', 'stop', 'inplay', 'our_penalty_inplay', 'their_penalty_inplay',
    'our_pre_kickoff', 'our_kickoff', 'their_pre_kickoff', 'their_kickoff',
    'our_pre_penalty', 'our_penalty', 'their_pre_penalty', 'their_penalty',
    'our_direct', 'their_direct', 'our_indirect', 'their_indirect',
    'our_timeout', 'their_timeout', 'our_ball_placement', 'their_ball_placement'
]
for name in function_names:
    setattr(DecisionBase, name, generate_function())

    # def halt(self, robot_id):
    #     operation_halt = Operation().halt()
    #     self._operator.operate(robot_id, operation_halt)

    # def stop(self, robot_id):
    #     operation_halt = Operation().halt()
    #     self._operator.operate(robot_id, operation_halt)

    # def inplay(self, robot_id):
    #     if self._act_id != self.ACT_ID_INPLAY:
    #         print("INPLAY:{}".format(robot_id))
    #         self._operator.stop(robot_id)
    #         self._act_id = self.ACT_ID_INPLAY

    # def our_penalty_inplay(self, robot_id):
    #     if self._act_id != self.ACT_ID_INPLAY:
    #         print("OUR PENALTY_INPLAY:{}".format(robot_id))
    #         self._operator.stop(robot_id)
    #         self._act_id = self.ACT_ID_INPLAY

    # def their_penalty_inplay(self, robot_id):
    #     if self._act_id != self.ACT_ID_INPLAY:
    #         print("THEIR PENALTY_INPLAY:{}".format(robot_id))
    #         self._operator.stop(robot_id)
    #         self._act_id = self.ACT_ID_INPLAY

    # def our_pre_kickoff(self, robot_id):
    #     if self._act_id != self.ACT_ID_PRE_KICKOFF:
    #         print("OUR PRE KICKOFF:{}".format(robot_id))
    #         self._operator.stop(robot_id)
    #         self._act_id = self.ACT_ID_PRE_KICKOFF

    # def our_kickoff(self, robot_id):
    #     if self._act_id != self.ACT_ID_KICKOFF:
    #         print("OUR KICKOFF:{}".format(robot_id))
    #         self._operator.stop(robot_id)
    #         self._act_id = self.ACT_ID_KICKOFF

    # def their_pre_kickoff(self, robot_id):
    #     if self._act_id != self.ACT_ID_PRE_KICKOFF:
    #         print("THEIR PRE KICKOFF:{}".format(robot_id))
    #         self._operator.stop(robot_id)
    #         self._act_id = self.ACT_ID_PRE_KICKOFF

    # def their_kickoff(self, robot_id):
    #     if self._act_id != self.ACT_ID_KICKOFF:
    #         print("THEIR KICKOFF:{}".format(robot_id))
    #         self._operator.stop(robot_id)
    #         self._act_id = self.ACT_ID_KICKOFF

    # def our_pre_penalty(self, robot_id):
    #     if self._act_id != self.ACT_ID_PRE_PENALTY:
    #         print("OUR PRE PENALTY:{}".format(robot_id))
    #         self._operator.stop(robot_id)
    #         self._act_id = self.ACT_ID_PRE_PENALTY

    # def our_penalty(self, robot_id):
    #     if self._act_id != self.ACT_ID_PENALTY:
    #         print("OUR PENALTY:{}".format(robot_id))
    #         self._operator.stop(robot_id)
    #         self._act_id = self.ACT_ID_PENALTY

    # def their_pre_penalty(self, robot_id):
    #     if self._act_id != self.ACT_ID_PRE_PENALTY:
    #         print("THEIR PRE PENALTY:{}".format(robot_id))
    #         self._operator.stop(robot_id)
    #         self._act_id = self.ACT_ID_PRE_PENALTY

    # def their_penalty(self, robot_id):
    #     if self._act_id != self.ACT_ID_PENALTY:
    #         print("THEIR penalty:{}".format(robot_id))
    #         self._operator.stop(robot_id)
    #         self._act_id = self.ACT_ID_PENALTY

    # def our_direct(self, robot_id):
    #     if self._act_id != self.ACT_ID_DIRECT:
    #         print("OUR DIRECT FREE KICK:{}".format(robot_id))
    #         self._operator.stop(robot_id)
    #         self._act_id = self.ACT_ID_DIRECT

    # def their_direct(self, robot_id):
    #     if self._act_id != self.ACT_ID_DIRECT:
    #         print("THEIR DIRECT FREE KICK:{}".format(robot_id))
    #         self._operator.stop(robot_id)
    #         self._act_id = self.ACT_ID_DIRECT

    # def our_indirect(self, robot_id):
    #     if self._act_id != self.ACT_ID_INDIRECT:
    #         print("OUR INDIRECT FREE KICK:{}".format(robot_id))
    #         self._operator.stop(robot_id)
    #         self._act_id = self.ACT_ID_INDIRECT

    # def their_indirect(self, robot_id):
    #     if self._act_id != self.ACT_ID_INDIRECT:
    #         print("THEIR INDIRECT FREE KICK:{}".format(robot_id))
    #         self._operator.stop(robot_id)
    #         self._act_id = self.ACT_ID_INDIRECT

    # def our_timeout(self, robot_id):
    #     if self._act_id != self.ACT_ID_TIMEOUT:
    #         self._operator.stop(robot_id)
    #         self._act_id = self.ACT_ID_TIMEOUT

    # def their_timeout(self, robot_id):
    #     if self._act_id != self.ACT_ID_TIMEOUT:
    #         self._operator.stop(robot_id)
    #         self._act_id = self.ACT_ID_TIMEOUT

    # def our_ball_placement(self, robot_id, placement_pos):
    #     if self._act_id != self.ACT_ID_OUR_PLACEMENT:
    #         print("OUR BALL PLACEMENT:{} to x:{}, y:{}".format(
    #             robot_id, placement_pos.x, placement_pos.y))
    #         self._operator.stop(robot_id)
    #         self._act_id = self.ACT_ID_OUR_PLACEMENT

    # def their_ball_placement(self, robot_id, placement_pos):
    #     if self._act_id != self.ACT_ID_THEIR_PLACEMENT:
    #         print("THEIR BALL PLACEMENT:{} to x:{}, y:{}".format(
    #             robot_id, placement_pos.x, placement_pos.y))
    #         self._operator.stop(robot_id)
    #         self._act_id = self.ACT_ID_THEIR_PLACEMENT
