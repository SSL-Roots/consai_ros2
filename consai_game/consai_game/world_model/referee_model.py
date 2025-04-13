#!/usr/bin/env python3
# coding: UTF-8

# Copyright 2025 Roots
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

from robocup_ssl_msgs.msg import Referee


class RefereeModel:
    def __init__(self, our_team_is_yellow: bool = False):
        self.our_team_is_yellow = our_team_is_yellow
        self.sub_referee = None
        self.current_command = Referee.COMMAND_HALT
        self.current_action_time_remaining = 0

    def parse_msg(self, msg: Referee):
        self.current_command = msg.command

        # current_action_time_remainingは
        # NORMAL STARTやFREE KICKなどのセットプレーで値が初期化され、カウントダウンが始まる
        if self.halt or self.stop:
            self.current_action_time_remaining = 0
        elif msg.current_action_time_remaining:
            self.current_action_time_remaining = msg.current_action_time_remaining[0]

    @property
    def halt(self):
        return self.current_command == Referee.COMMAND_HALT

    @property
    def stop(self):
        return self.current_command == Referee.COMMAND_STOP

    @property
    def force_start(self):
        return self.current_command == Referee.COMMAND_FORCE_START

    @property
    def normal_start(self):
        return self.current_command == Referee.COMMAND_NORMAL_START

    @property
    def our_free_kick(self):
        return (
            self.current_command == Referee.COMMAND_DIRECT_FREE_BLUE
            and self.our_team_is_yellow is False
        ) or (
            self.current_command == Referee.COMMAND_DIRECT_FREE_YELLOW
            and self.our_team_is_yellow is True
        )

    @property
    def their_free_kick(self):
        return (
            self.current_command == Referee.COMMAND_DIRECT_FREE_YELLOW
            and self.our_team_is_yellow is False
        ) or (
            self.current_command == Referee.COMMAND_DIRECT_FREE_BLUE
            and self.our_team_is_yellow is True
        )

    @property
    def our_kick_off(self):
        return (
            self.current_command == Referee.COMMAND_PREPARE_KICKOFF_BLUE
            and self.our_team_is_yellow is False
        ) or (
            self.current_command == Referee.COMMAND_PREPARE_KICKOFF_YELLOW
            and self.our_team_is_yellow is True
        )

    @property
    def their_kick_off(self):
        return (
            self.current_command == Referee.COMMAND_PREPARE_KICKOFF_YELLOW
            and self.our_team_is_yellow is False
        ) or (
            self.current_command == Referee.COMMAND_PREPARE_KICKOFF_BLUE
            and self.our_team_is_yellow is True
        )

    @property
    def our_penalty_kick(self):
        return (
            self.current_command == Referee.COMMAND_PREPARE_PENALTY_BLUE
            and self.our_team_is_yellow is False
        ) or (
            self.current_command == Referee.COMMAND_PREPARE_PENALTY_YELLOW
            and self.our_team_is_yellow is True
        )

    @property
    def their_penalty_kick(self):
        return (
            self.current_command == Referee.COMMAND_PREPARE_PENALTY_YELLOW
            and self.our_team_is_yellow is False
        ) or (
            self.current_command == Referee.COMMAND_PREPARE_PENALTY_BLUE
            and self.our_team_is_yellow is True
        )

    @property
    def our_goal(self):
        return (
            self.current_command == Referee.COMMAND_GOAL_BLUE
            and self.our_team_is_yellow is False
        ) or (
            self.current_command == Referee.COMMAND_GOAL_YELLOW
            and self.our_team_is_yellow is True
        )

    @property
    def their_goal(self):
        return (
            self.current_command == Referee.COMMAND_GOAL_YELLOW
            and self.our_team_is_yellow is False
        ) or (
            self.current_command == Referee.COMMAND_GOAL_BLUE
            and self.our_team_is_yellow is True
        )

    @property
    def our_timeout(self):
        return (
            self.current_command == Referee.COMMAND_TIMEOUT_BLUE
            and self.our_team_is_yellow is False
        ) or (
            self.current_command == Referee.COMMAND_TIMEOUT_YELLOW
            and self.our_team_is_yellow is True
        )

    @property
    def their_timeout(self):
        return (
            self.current_command == Referee.COMMAND_TIMEOUT_YELLOW
            and self.our_team_is_yellow is False
        ) or (
            self.current_command == Referee.COMMAND_TIMEOUT_BLUE
            and self.our_team_is_yellow is True
        )

    @property
    def our_ball_placement(self):
        return (
            self.current_command == Referee.COMMAND_BALL_PLACEMENT_BLUE
            and self.our_team_is_yellow is False
        ) or (
            self.current_command == Referee.COMMAND_BALL_PLACEMENT_YELLOW
            and self.our_team_is_yellow is True
        )

    @property
    def their_ball_placement(self):
        return (
            self.current_command == Referee.COMMAND_BALL_PLACEMENT_YELLOW
            and self.our_team_is_yellow is False
        ) or (
            self.current_command == Referee.COMMAND_BALL_PLACEMENT_BLUE
            and self.our_team_is_yellow is True
        )

    @property
    def running(self):
        return self.current_action_time_remaining < 0
