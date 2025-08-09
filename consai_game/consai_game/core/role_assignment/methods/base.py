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

"""ロボットの役割割り当て手法のインターフェースを定義するモジュール."""

from abc import ABC, abstractmethod
from typing import List

from consai_game.core.tactic.tactic_base import TacticBase
from consai_game.world_model.world_model import WorldModel


class RoleAssignmentBase(ABC):
    """役割割り当ての基底クラス."""

    @abstractmethod
    def assign(
        self,
        play_roles: List[List[TacticBase]],
        world_model: WorldModel,
        goalie_id: int,
    ) -> List[int]:
        """Playファイルのrolesをもとに、ロボットのIDリストを返す."""
        raise NotImplementedError()
