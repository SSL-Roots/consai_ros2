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

"""ロボットの現在位置とTacticの目標位置の距離に基づいて役割を割り当てる手法を定義するモジュール."""

import numpy as np
from scipy.optimize import linear_sum_assignment
from copy import deepcopy
from typing import List

from consai_msgs.msg import MotionCommand
from consai_tools.geometry.geometry_tools import get_distance

from consai_game.core.role_assignment.methods.base import RoleAssignmentBase
from consai_game.core.tactic.role import RoleConst
from consai_game.core.tactic.tactic_base import TacticBase
from consai_game.world_model.world_model import WorldModel


class ByCost(RoleAssignmentBase):
    """
    コストが最小になるようにRoleを割り当てる.

    コストはTactic実行時に生成される目標位置とロボットの距離とする
    """

    def __init__(self):
        """ByCostの初期化処理."""
        super().__init__()
        self.GAIN_TARGET_POSE = 1.0

    def assign(
        self,
        play_roles: List[List[TacticBase]],
        world_model: WorldModel,
        goalie_id: int,
    ) -> List[int]:
        """コストが最小になるようにロールを割り当てる関数."""
        role_num = len(play_roles)
        id_list = [RoleConst.INVALID_ROLE_ID] * role_num

        # goalie_idが存在すれば先頭にセットする
        if goalie_id in world_model.robot_activity.our_visible_robots:
            id_list[0] = goalie_id

        # goalieを除いたIDリスト
        robots_without_goalie = [
            robot_id for robot_id in world_model.robots.our_visible_robots.keys() if robot_id != goalie_id
        ]

        # ロボットがいなければ終了
        if len(robots_without_goalie) == 0:
            return id_list

        # ゴーリー(play[0])を除き、ロボットの台数だけplay_rolesを用意する
        # 後段でtacticを実行するため、deepcopyする
        play_roles_without_goalie = deepcopy(play_roles[1 : len(robots_without_goalie) + 1])  # noqa: E203

        # ゴーリーを除いたコスト行列を作成
        # NOTE:ロボットの台数が少ない場合、末端のroleは切り捨てられる
        cost_matrix = np.zeros((len(robots_without_goalie), len(play_roles_without_goalie)))

        for i, robot_id in enumerate(robots_without_goalie):
            # ロボットの位置
            robot_pos = world_model.robots.our_visible_robots[robot_id].pos

            for j, tactic in enumerate(play_roles_without_goalie):
                if len(tactic) == 0:
                    # tacticが空の場合はコストを最大にする
                    cost_matrix[i, j] = float("inf")
                    continue

                # tacticを実行する
                tactic[0].reset(robot_id)
                command = tactic[0].run(world_model)

                if command.mode == MotionCommand.MODE_DIRECT_VELOCITY:
                    # 速度制御の場合は、コストを0にする
                    cost_target = 0.0
                else:
                    # 目標位置との距離コスト
                    cost_target = self.GAIN_TARGET_POSE * get_distance(robot_pos, command.desired_pose)

                cost_matrix[i, j] = cost_target

        # 最適化問題を解く
        robot_index, role_index = linear_sum_assignment(cost_matrix)

        # id_listにロボットIDをセットする
        # id_list[0]はゴーリーのため、robot_indexの先頭から1つずらす
        for i, robot in enumerate(robot_index):
            id_list[role_index[i] + 1] = robots_without_goalie[robot]

        return id_list
