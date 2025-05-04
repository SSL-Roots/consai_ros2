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

from consai_msgs.msg import MotionCommand

from consai_game.world_model.world_model import WorldModel
from consai_game.core.tactic.tactic_base import TacticBase


class SlowSafe(TacticBase):
    """ボールを避けながら低速で移動するWrapperTactic."

    SlowSafe(tactic=Position()) のように使用する
    """

    MAX_VELOCITY = 1.0  # 最大速度の制限値 [m/s]
    AVOID_RADIUS = 0.6  # ボールを避ける半径 [m]

    def __init__(self, tactic=TacticBase):
        """inner_tacticを初期化する関数."""
        super().__init__()
        self.inner_tactic = tactic

    def reset(self, robot_id: int) -> None:
        """inner_tacticをリセットする関数."""
        super().reset(robot_id)
        self.inner_tactic.reset(robot_id)

    def exit(self):
        """inner_tacticをexitする関数."""
        super().exit()
        self.inner_tactic.exit()

    def run(self, world_model: WorldModel) -> MotionCommand:
        """ディフェンスエリア内での移動と、ボールとの接触を許可する."""
        command = self.inner_tactic.run(world_model)

        # 最大速度を制限
        command.desired_velocity.x = self.MAX_VELOCITY
        command.desired_velocity.y = self.MAX_VELOCITY

        # NaviOptionsを設定してボールを避ける
        command.navi_options.avoid_ball = True
        command.navi_options.ball_avoid_radius = self.AVOID_RADIUS
        command.navi_options.avoid_our_robots = True
        command.navi_options.avoid_their_robots = True
        command.navi_options.avoid_pushing = True
        command.navi_options.avoid_defense_area = True

        # stateを上書きする
        self.state = self.inner_tactic.state

        return command
