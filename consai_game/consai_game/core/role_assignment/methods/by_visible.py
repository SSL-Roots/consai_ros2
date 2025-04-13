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

from consai_game.core.role_assignment.methods.base import RoleAssignmentBase
from consai_game.core.tactic.role import RoleConst
from consai_game.core.tactic.tactic_base import TacticBase
from consai_game.world_model.world_model import WorldModel
from typing import List


class ByVisible(RoleAssignmentBase):
    """フィールドに現れた順にRoleを割り当てる."""

    def assign(
        self, play_roles: List[List[TacticBase]], world_model: WorldModel
    ) -> List[int]:
        # ロールの数だけ、ロボットIDを取得する
        id_list = world_model.robot_activity.ordered_our_visible_robots[
            : len(play_roles)
        ]

        # ロボットの数がロールより少ない場合、INVALID_ROLE_IDで埋める
        id_list += [RoleConst.INVALID_ROLE_ID] * (len(play_roles) - len(id_list))
        return id_list
