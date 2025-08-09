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

from consai_game.core.tactic.tactic_base import TacticBase


class WrapperTacticBase(TacticBase):
    """WrapperTacticの基底クラス.

    WrapperTacticは、他のTacticをラップして、特定の機能を追加する
    """

    def __init__(self, tactic=TacticBase):
        """内部tacticを初期化する関数."""
        super().__init__()
        self.inner_tactic = tactic
        self.name += f"({self.inner_tactic.name})"

    def reset(self, robot_id: int) -> None:
        """inner_tacticをリセットする関数."""
        super().reset(robot_id)
        self.inner_tactic.reset(robot_id)

    def exit(self):
        """inner_tacticをexitする関数."""
        super().exit()
        self.inner_tactic.exit()
