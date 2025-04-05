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

from consai_game.core.tactic.role import Role
from consai_game.core.tactic.tactic_base import TacticState
from consai_msgs.msg import MotionCommand
from typing import Optional


class Agent():
    def __init__(self):

        self.role = Role()
        self.present_tactic = None
        self.present_tactic_index = 0

    def update(self) -> Optional[MotionCommand]:
        if self.present_tactic is None:
            return None

        # Tacticの処理が終了していたら、次のTacticに移る
        if self.present_tactic.state == TacticState.FINISHED:
            self.present_tactic_index += 1

            # すべてのTacticを実行したら、最初のTacticに戻る
            if self.present_tactic_index >= len(self.role.tactics):
                self.present_tactic_index = 0
            self.present_tactic = self.role.tactics[self.present_tactic_index]

        if self.present_tactic.state == TacticState.BEFORE_INIT:
            self.present_tactic.reset(self.role.robot_id)

        return self.present_tactic.run()

    def set_role(self, role: Role) -> None:
        self.role = role
        if len(self.role.tactics) == 0:
            raise ValueError("Tactics is empty")

        if self.role.robot_id < 0:
            raise ValueError("Robot ID is not set")

        self.present_tactic_index = 0
        self.present_tactic = self.role.tactics[self.present_tactic_index]()
