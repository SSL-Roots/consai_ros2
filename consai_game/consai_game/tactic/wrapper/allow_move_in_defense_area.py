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

from consai_game.core.tactic.wrapper_tactic_base import WrapperTacticBase
from consai_game.world_model.world_model import WorldModel


class AllowMoveInDefenseArea(WrapperTacticBase):
    """ディフェンスエリア内での移動を許可するWrapperTactic."

    AllowMoveInDefenseArea(tactic=Position()) のように使用する
    """

    def run(self, world_model: WorldModel) -> MotionCommand:
        """ディフェンスエリア内での移動と、ボールとの接触を許可する."""
        command = self.inner_tactic.run(world_model)

        # ボールを回避をしない
        command.navi_options.avoid_ball = False
        # ディフェンスエリアを回避をしない
        command.navi_options.avoid_defense_area = False

        # stateを上書きする
        self.state = self.inner_tactic.state

        return command
