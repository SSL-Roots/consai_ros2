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
BallClear Tactic.

自ディフェンスエリアにあるボールをクリアする.
"""

from consai_game.core.tactic.tactic_base import TacticBase
from consai_game.tactic.kick.kick import Kick
from consai_game.world_model.world_model import WorldModel

from consai_msgs.msg import MotionCommand
from consai_msgs.msg import State2D


class BallClear(TacticBase):
    """自ディフェンスエリアにあるボールをクリアするTactic."""

    def __init__(self):
        """Initialize the DefendGoal tactic."""
        super().__init__()
        self.kick_tactic = Kick(x=6.0, y=0.0, is_pass=False, is_setplay=False)

    def reset(self, robot_id: int) -> None:
        """Reset the tactic state for the specified robot."""
        self.robot_id = robot_id
        self.kick_tactic.reset(robot_id)

    def run(self, world_model: WorldModel, x=0.0, y=3.5) -> MotionCommand:
        """Run the tactic and return a MotionCommand based on the ball's position and movement."""

        # ロボットの位置を取得
        robot_pos = world_model.robots.our_robots.get(self.robot_id).pos

        # キックターゲットを取得
        # TODO: いつかパスをやるようにしたいのでコメントアウトとして残す
        # kick_target_model = world_model.kick_target
        # if (
        #     kick_target_model.best_pass_target.success_rate > 30
        #     and kick_target_model.best_pass_target.robot_id != -1
        # ):
        #     # パスターゲットの位置を取得
        #     target_pos = kick_target_model.best_pass_target.robot_pos
        # else:

        # ボールクリアの位置を設定
        target_pos = State2D()
        if robot_pos.y < 0.0:
            target_pos.y = -y
        else:
            target_pos.y = y
        target_pos.x = x

        self.kick_tactic.target_pos = target_pos

        return self.kick_tactic.run(world_model)
