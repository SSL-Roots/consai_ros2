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
条件に応じてディフェンス動作やキックやパスを切り替えるTactic
"""

from consai_game.core.tactic.tactic_base import TacticBase
from consai_game.world_model.world_model import WorldModel
from consai_game.tactic.circular_move import CircularMove

from consai_msgs.msg import MotionCommand


class BallCentricMover(TacticBase):
    def __init__(self, radius=0.3, seconds=20, cw=True):
        super().__init__()
        self.circular_move = CircularMove(radius=radius, seconds=seconds, cw=cw)

    def reset(self, robot_id: int) -> None:
        """Reset the tactic state for the specified robot."""
        super().reset(robot_id)

        # 所有するTacticも初期化する
        self.circular_move.reset(robot_id)

    def exit(self):
        super().exit()

        # 所有するTacticもexitする
        self.circular_move.exit()

    def run(self, world_model: WorldModel) -> MotionCommand:
        """状況に応じて実行するtacticを切り替えてrunする."""

        ball_pos = world_model.ball.pos

        # ボールに近くない場合はデフォルトのtacticを実行する
        return self.circular_move.run(world_model, x=ball_pos.x, y=ball_pos.y)
