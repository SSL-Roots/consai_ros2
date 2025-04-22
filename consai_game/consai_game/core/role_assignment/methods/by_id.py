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

"""ロボットIDの昇順に基づいて役割を割り当てる手法を定義するモジュール."""

from typing import List

from consai_game.core.role_assignment.methods.base import RoleAssignmentBase
from consai_game.core.tactic.role import RoleConst
from consai_game.core.tactic.tactic_base import TacticBase
from consai_game.world_model.world_model import WorldModel


class ByIDMethod(RoleAssignmentBase):
    """ID昇順にRoleを割り当てるクラス."""

    def assign(
        self,
        play_roles: List[List[TacticBase]],
        world_model: WorldModel,
        goalie_id: int,
    ) -> List[int]:
        """ID昇順でロールを割り当てる関数."""
        role_num = len(play_roles)

        id_list = [RoleConst.INVALID_ROLE_ID] * role_num

        # goalie_idが存在すれば先頭にセットする
        if goalie_id in world_model.robot_activity.our_visible_robots:
            id_list[0] = goalie_id

        # goalie_id以外のロボットIDを抽出する
        visible_robots = [
            robot_id for robot_id in world_model.robot_activity.our_visible_robots if robot_id != goalie_id
        ]

        # id_listにロボットIDをセットする
        for i in range(1, role_num):
            if len(visible_robots) == 0:
                break
            id_list[i] = visible_robots.pop(0)

        return id_list
