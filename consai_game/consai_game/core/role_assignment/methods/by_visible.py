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

"""視認可能なロボット順に基づいて役割を割り当てる手法を定義するモジュール."""

from typing import List

from consai_game.core.role_assignment.methods.base import RoleAssignmentBase
from consai_game.core.tactic.role import RoleConst
from consai_game.core.tactic.tactic_base import TacticBase
from consai_game.world_model.world_model import WorldModel


class ByVisible(RoleAssignmentBase):
    """
    できるだけ割当を維持しつつ、フィールドに現れた順にRoleを割り当てる.

    前回の割当を記憶し、ロボットが消えたRoleには、最後に登場したロボットを割り当てる.
    """

    def __init__(self):
        """割当履歴を初期化するコンストラクタ."""
        super().__init__()
        self.prev_id_list: List[int] = []

    def assign(
        self,
        play_roles: List[List[TacticBase]],
        world_model: WorldModel,
        goalie_id: int,
    ) -> List[int]:
        """視認順と前回割当をもとにロールを割り当てる関数."""
        role_num = len(play_roles)

        if len(self.prev_id_list) == role_num:
            # 前回の割当と今回のロール数が同じであれば、前回の割当をベースにする
            id_list = self.prev_id_list
        else:
            # 数が異なれば初期化する
            id_list = [RoleConst.INVALID_ROLE_ID] * role_num

        # goalie_idが存在すれば先頭にセットする
        if goalie_id in world_model.robot_activity.ordered_our_visible_robots:
            id_list[0] = goalie_id

        # ロボットが消えた役割を初期化する
        for i, robot_id in enumerate(id_list):
            if robot_id not in world_model.robot_activity.ordered_our_visible_robots:
                id_list[i] = RoleConst.INVALID_ROLE_ID

        # 新しく現れたロボットを抽出する
        new_robots = [
            robot_id for robot_id in world_model.robot_activity.ordered_our_visible_robots if robot_id not in id_list
        ]

        # 新しく現れたロボットを、空いている役割に割り当てる
        for new_robot_id in new_robots:
            for i in range(1, role_num):
                if id_list[i] == RoleConst.INVALID_ROLE_ID:
                    id_list[i] = new_robot_id
                    break

        # 割当を記憶する
        self.prev_id_list = id_list.copy()

        return id_list
