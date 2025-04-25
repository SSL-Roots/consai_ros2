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

"""シュート動作に関するTacticを定義するモジュール."""

from consai_game.world_model.world_model import WorldModel
from consai_game.core.tactic.tactic_base import TacticBase, TacticState
from consai_game.tactic.kick.kick import Kick
from consai_msgs.msg import MotionCommand, State2D


class Shoot(TacticBase):
    """シュート動作を行うTactic"""

    def __init__(self):
        """シュートのTacticインスタンスを初期化する関数."""
        super().__init__()

        self.kick_tactic = Kick(x=6.0, y=0.0, is_pass=False)

    def reset(self, robot_id: int) -> None:
        """ロボットIDを設定し, Tacticの状態をRUNNINGにリセットする関数."""
        self.robot_id = robot_id
        self.state = TacticState.RUNNING
        self.kick_tactic.reset(robot_id)

    def run(self, world_model: WorldModel) -> MotionCommand:
        """シュートを実行するためのMotionCommandを生成する関数."""

        # キックターゲットを取得
        kick_target_model = world_model.kick_target
        if kick_target_model.best_shoot_target.success_rate > 50:
            # シュートターゲットの位置を取得
            target_pos = kick_target_model.best_shoot_target.pos
        elif kick_target_model.best_pass_target.success_rate > 50 and kick_target_model.best_pass_target.robot_id != -1:
            # パスターゲットの位置を取得
            target_pos = kick_target_model.best_pass_target.robot_pos
        else:
            # デフォルトのシュートターゲットの位置を設定
            target_pos = State2D()
            target_pos.x = 6.0
            target_pos.y = 0.0

        self.kick_tactic.target_pos = target_pos

        return self.kick_tactic.run(world_model)
