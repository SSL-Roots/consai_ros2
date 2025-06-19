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

"""
チップキックのTactic.
"""

from consai_game.core.tactic.tactic_base import TacticBase
from consai_game.tactic.kick import Kick
from consai_game.world_model.world_model import WorldModel

from consai_msgs.msg import MotionCommand
from consai_msgs.msg import State2D


class ChipKick(TacticBase):
    """チップキックをするTactic."""

    def __init__(self):
        """初期化関数."""
        super().__init__()
        self.kick_tactic = Kick(x=6.0, y=0.0, is_pass=False, is_setplay=False)

    def reset(self, robot_id: int) -> None:
        """タクティックの状態をリセットする関数."""
        super().reset(robot_id)
        self.robot_id = robot_id
        self.kick_tactic.reset(robot_id)

    def run(self, world_model: WorldModel, x=0.0, y=0.0) -> MotionCommand:
        """チップキックを行い、MotionCommandを返す関数."""

        # 蹴る目標位置を設定
        target_pos = State2D()
        target_pos.x = x
        target_pos.y = y

        self.kick_tactic.target_pos = target_pos
        command = self.kick_tactic.run(world_model)

        # チップキックを行うよう設定
        command.chip_kick = True
        
        return command 
