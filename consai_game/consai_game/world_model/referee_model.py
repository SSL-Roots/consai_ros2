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

    def callback(self, msg: Referee):
        self.current_command = msg.command

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
    def free_kick_blue(self):
        return self.current_command == Referee.COMMAND_DIRECT_FREE_BLUE
    
    @property
    def free_kick_yellow(self):
        return self.current_command == Referee.COMMAND_DIRECT_FREE_YELLOW
    
    @property
    def kick_off_blue(self):
        return self.current_command == Referee.COMMAND_PREPARE_KICKOFF_BLUE
    
    @property
    def kick_off_yellow(self):
        return self.current_command == Referee.COMMAND_PREPARE_KICKOFF_YELLOW
    
    @property
    def penalty_kick_blue(self):
        return self.current_command == Referee.COMMAND_PREPARE_PENALTY_BLUE
    
    @property
    def penalty_kick_yellow(self):
        return self.current_command == Referee.COMMAND_PREPARE_PENALTY_YELLOW
    
    @property
    def goal_blue(self):
        return self.current_command == Referee.COMMAND_GOAL_BLUE
    
    @property
    def goal_yellow(self):
        return self.current_command == Referee.COMMAND_GOAL_YELLOW
    
    @property
    def timeout_blue(self):
        return self.current_command == Referee.COMMAND_TIMEOUT_BLUE
    
    @property
    def timeout_yellow(self):
        return self.current_command == Referee.COMMAND_TIMEOUT_YELLOW
    
    @property
    def ball_placement_blue(self):
        return self.current_command == Referee.COMMAND_BALL_PLACEMENT_BLUE
    
    @property
    def ball_placement_yellow(self):
        return self.current_command == Referee.COMMAND_BALL_PLACEMENT_YELLOW