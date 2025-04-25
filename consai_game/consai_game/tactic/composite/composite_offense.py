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
条件に応じてキックやパスを切り替えるTactic
"""

from consai_game.core.tactic.tactic_base import TacticBase, TacticState
from consai_game.tactic.kick.kick import Kick
from consai_game.world_model.world_model import WorldModel

from consai_msgs.msg import MotionCommand


class CompositeOffense(TacticBase):
    def __init__(self, tactic_default: TacticBase):
        super().__init__()
        self.tactic_shoot = Kick(is_pass=False)
        self.tactic_pass = Kick(is_pass=True)
        self.tactic_default = tactic_default

    def reset(self, robot_id: int) -> None:
        """Reset the tactic state for the specified robot."""
        self.robot_id = robot_id
        self.state = TacticState.RUNNING

        # 所有するTacticも初期化する
        self.tactic_shoot.reset(robot_id)
        self.tactic_pass.reset(robot_id)
        self.tactic_default.reset(robot_id)

    def run(self, world_model: WorldModel) -> MotionCommand:
        if world_model.robot_activity.our_robots_by_ball_distance[0] == self.robot_id:
            # ボールに近い場合はボールを操作する
            return self.control_the_ball(world_model)

        # ボールに近くない場合はデフォルトのtacticを実行する
        return self.tactic_default.run(world_model)

    def control_the_ball(self, world_model: WorldModel) -> MotionCommand:
        """ボールを制御するためのTacticを実行する関数."""

        if world_model.kick_target.best_shoot_target.success_rate > 50:
            # シュートできる場合
            self.tactic_shoot.target_pos = world_model.kick_target.best_shoot_target.pos
            return self.tactic_shoot.run(world_model)

        elif world_model.kick_target.best_pass_target.success_rate > 50:
            # パスできる場合
            self.tactic_pass.target_pos = world_model.kick_target.best_pass_target.robot_pos
            return self.tactic_pass.run(world_model)

        # TODO: コツコツドリブルを実行する
        self.tactic_pass.target_pos = world_model.robots.our_visible_robots[self.robot_id].pos
        return self.tactic_pass.run(world_model)
