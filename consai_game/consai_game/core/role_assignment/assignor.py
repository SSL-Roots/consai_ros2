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

"""役割割り当て手法を用いてプレイのロールにロボットIDを割り当てるモジュール."""

from typing import List

from consai_game.core.role_assignment.methods.base import RoleAssignmentBase
from consai_game.core.tactic.tactic_base import TacticBase
from consai_game.world_model.world_model import WorldModel


class RoleAssignor:
    """ロール割り当て手法に基づきロボットIDを決定するクラス."""

    def __init__(self, method: RoleAssignmentBase, goalie_id: int):
        """割り当て手法とゴールキーパーIDを初期化する関数."""
        self.method = method
        self.goalie_id = goalie_id

    def assign(self, play_roles: List[List[TacticBase]], world_model: WorldModel) -> List[int]:
        """プレイロールとワールドモデルをもとにロボットIDを割り当てる関数."""
        return self.method.assign(play_roles=play_roles, world_model=world_model, goalie_id=self.goalie_id)
