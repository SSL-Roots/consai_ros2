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

from consai_game.core.tactic.role import Role, RoleConst
from consai_game.core.tactic.tactic_base import TacticState
from consai_game.world_model.world_model import WorldModel
from consai_msgs.msg import MotionCommand
from typing import Optional


class Agent():
    def __init__(self):

        self.role = Role()
        self.present_tactic = None
        self.present_tactic_index = 0

    def update(self, world_model: WorldModel) -> Optional[MotionCommand]:
        if self.present_tactic is None:
            return None

        if self.role.robot_id == RoleConst.INVALID_ROLE_ID:
            return None

        # Tacticの処理が終了していたら、次のTacticに移る
        if self.present_tactic.state == TacticState.FINISHED:
            self.present_tactic_index += 1

            # すべてのTacticを実行したら、最初のTacticに戻る
            if self.present_tactic_index >= len(self.role.tactics):
                self.present_tactic_index = 0
            self.reset_tactic()

        return self.present_tactic.run(world_model=world_model)

    def set_role(self, role: Role) -> None:
        self.role = role
        if len(self.role.tactics) == 0:
            raise ValueError("Tactics is empty")

        if not (RoleConst.MIN_VALID_ROLE_ID <= self.role.robot_id <= RoleConst.MAX_VALID_ROLE_ID):
            if self.role.robot_id != RoleConst.INVALID_ROLE_ID:
                raise ValueError(f"Invalid robot_id: {self.role.robot_id}")

        self.present_tactic_index = 0
        self.reset_tactic()

    def reset_tactic(self) -> None:
        self.present_tactic = self.role.tactics[self.present_tactic_index]
        self.present_tactic.reset(self.role.robot_id)
