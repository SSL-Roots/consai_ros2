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
from consai_game.world_model.world_model import WorldModel
from consai_msgs.msg import MotionCommand


class CompositeTacticBase(TacticBase):
    def __init__(self):
        super().__init__()

    def run_sub_tactic(self, tactic: TacticBase, world_model: WorldModel) -> MotionCommand:
        """サブTacticを実行し、nameを自動更新する"""
        command = tactic.run(world_model)
        self.name = f"{self.__class__.__name__}({tactic.name})"
        return command
