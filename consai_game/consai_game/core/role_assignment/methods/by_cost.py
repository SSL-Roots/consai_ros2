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
from consai_tools.geometry.geometry_tools import get_distance
from scipy.optimize import linear_sum_assignment
from typing import List
import numpy as np


class ByCost(RoleAssignmentBase):
    """
    コストが最小になるようにRoleを割り当てる.

    コストはTactic実行時に生成される目標位置とロボットの距離とする
    """

    def __init__(self):
        super().__init__()
        self.GAIN_TARGET_POSE = 100
        self.GAIN_BALL = 0.5

    def assign(
        self,
        play_roles: List[List[TacticBase]],
        world_model: WorldModel,
        goalie_id: int,
    ) -> List[int]:

        role_num = len(play_roles)
        id_list = [RoleConst.INVALID_ROLE_ID] * role_num

        # goalie_idが存在すれば先頭にセットする
        if goalie_id in world_model.robot_activity.our_visible_robots:
            id_list[0] = goalie_id

        # goalieを除いたIDリスト
        robots_without_goalie = [
            robot_id for robot_id in world_model.robot_activity.our_visible_robots if robot_id != goalie_id
        ]

        # ロボットがいなければ終了
        if len(robots_without_goalie) == 0:
            return id_list

        # ゴーリーを除いたコスト行列を作成
        cost_matrix = np.zeros((len(robots_without_goalie), role_num - 1))

        for i, robot_id in enumerate(robots_without_goalie):
            # ロボットの位置
            # TODO: 例外処理を入れたほうが良い
            robot_pos = world_model.robots.our_robots[robot_id].pos

            # Role0はgoalieのため除く
            for j, tactic in enumerate(play_roles[1:]):
                # TODO: 例外処理を入れたほうが良い
                tactic[0].reset(robot_id)

                target_pos = tactic[0].run(world_model).desired_pose

                # 目標位置との距離コスト
                cost_target = self.GAIN_TARGET_POSE * get_distance(robot_pos, target_pos)
                # ボールとの距離コスト
                cost_ball = self.GAIN_BALL * get_distance(world_model.ball.pos, robot_pos)

                cost_matrix[i, j] = cost_target + cost_ball

        # 最適化問題を解く
        robot_index, role_index = linear_sum_assignment(cost_matrix)

        # デバッグ表示
        for robot, role in zip(robot_index, role_index):
            print(f"robot {robots_without_goalie[robot]} -> role {role}")

        # id_listにロボットIDをセットする
        for i, robot in enumerate(robot_index):
            id_list[role_index[i] + 1] = robots_without_goalie[robot]

        # for robot_id in world_model.robot_activity.our_visible_robots:

        #     if robot_id not in id_list:
        #         # goalie_idが存在しない場合は、最初にセットする
        #         if id_list[0] == RoleConst.INVALID_ROLE_ID:
        #             id_list[0] = robot_id
        #         break

        # # 新しく現れたロボットを抽出する
        # new_robots = [
        #     robot_id
        #     for robot_id in world_model.robot_activity.ordered_our_visible_robots
        #     if robot_id not in id_list
        # ]

        # # 新しく現れたロボットを、空いている役割に割り当てる
        # for new_robot_id in new_robots:
        #     for i in range(1, role_num):
        #         if id_list[i] == RoleConst.INVALID_ROLE_ID:
        #             id_list[i] = new_robot_id
        #             break

        # # 割当を記憶する
        # self.prev_id_list = id_list.copy()

        return id_list
