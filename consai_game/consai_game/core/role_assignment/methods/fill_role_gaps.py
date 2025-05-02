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

from typing import List

from consai_game.core.tactic.role import RoleConst


def fill_role_gaps(id_list: List[int], ignore_index: int) -> List[int]:
    """空いている役割を埋める関数.

    インデックスの先頭ほど優先度が高い.
    優先度の低いもの（配列の末尾）から順に割当を変えていく.
    """
    fill_candidate_idx = len(id_list) - 1

    for empty_idx in range(len(id_list)):
        if empty_idx == ignore_index:
            continue

        if id_list[empty_idx] == RoleConst.INVALID_ROLE_ID:
            # 後ろから探して、robot_idが設定されているものを探す
            while fill_candidate_idx > empty_idx and id_list[fill_candidate_idx] == RoleConst.INVALID_ROLE_ID:
                fill_candidate_idx -= 1

            if fill_candidate_idx > empty_idx:
                id_list[empty_idx] = id_list[fill_candidate_idx]
                id_list[fill_candidate_idx] = -1
                fill_candidate_idx -= 1

    return id_list
