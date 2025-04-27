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
from consai_game.tactic.receive import Receive
from consai_game.world_model.world_model import WorldModel


from consai_msgs.msg import MotionCommand


class CompositeOffense(TacticBase):
    def __init__(self, tactic_default: TacticBase):
        super().__init__()
        self.tactic_shoot = Kick(is_pass=False)
        self.tactic_pass = Kick(is_pass=True)
        self.tactic_tapping = Kick(is_tapping=True)
        self.tactic_receive = Receive()
        self.tactic_default = tactic_default

    def reset(self, robot_id: int) -> None:
        """Reset the tactic state for the specified robot."""
        self.robot_id = robot_id
        self.state = TacticState.RUNNING

        # 所有するTacticも初期化する
        self.tactic_shoot.reset(robot_id)
        self.tactic_pass.reset(robot_id)
        self.tactic_tapping.reset(robot_id)
        self.tactic_receive.reset(robot_id)
        self.tactic_default.reset(robot_id)

    def run(self, world_model: WorldModel) -> MotionCommand:
        """状況に応じて実行するtacticを切り替えてrunする."""

        # ボールが動いている場合は、自分がレシーブできるかを判断する
        best_receive_score = world_model.robot_activity.our_ball_receive_score[0]
        if (
            world_model.ball_activity.ball_is_moving
            and best_receive_score.robot_id == self.robot_id
            and best_receive_score.intercept_time != float("inf")
        ):
            # 自分がレシーブできる場合
            return self.tactic_receive.run(world_model)

        elif world_model.robot_activity.our_robots_by_ball_distance[0] == self.robot_id:
            # ボールに一番近い場合はボールを操作する
            return self.control_the_ball(world_model)

        # ボールに近くない場合はデフォルトのtacticを実行する
        return self.tactic_default.run(world_model)

    def control_the_ball(self, world_model: WorldModel) -> MotionCommand:
        """ボールを制御するためのTacticを実行する関数."""

        if world_model.kick_target.best_shoot_target.success_rate > 50:
            # シュートできる場合
            self.tactic_shoot.target_pos = world_model.kick_target.best_shoot_target.pos
            return self.tactic_shoot.run(world_model)

        elif world_model.kick_target.best_pass_target.success_rate > 30:
            # パスできる場合
            self.tactic_pass.target_pos = world_model.kick_target.best_pass_target.robot_pos
            return self.tactic_pass.run(world_model)

        # TODO: 前進しつつ、敵がいない方向にドリブルしたい
        # シュート成功率が一番高いところに向かってドリブルする
        self.tactic_tapping.target_pos = world_model.kick_target.best_shoot_target.pos
        return self.tactic_tapping.run(world_model)
