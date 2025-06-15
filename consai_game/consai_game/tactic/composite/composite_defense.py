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

from consai_game.core.tactic.composite_tactic_base import CompositeTacticBase
from consai_game.core.tactic.tactic_base import TacticBase
from consai_game.tactic.kick import Kick
from consai_game.tactic.receive import Receive
from consai_game.world_model.world_model import WorldModel
from consai_tools.geometry import geometry_tools as tool

from consai_msgs.msg import MotionCommand


class CompositeDefense(CompositeTacticBase):
    def __init__(self, tactic_default: TacticBase, do_receive: bool = True):
        super().__init__(
            tactic_shoot=Kick(is_pass=False),
            tactic_pass=Kick(is_pass=True),
            tactic_receive=Receive(),
            tactic_default=tactic_default,
        )
        self.diff_goal_threshold = 2.5
        self.very_close_to_ball_threshold = 0.3
        self.do_receive = do_receive

    def run(self, world_model: WorldModel) -> MotionCommand:
        """状況に応じて実行するtacticを切り替えてrunする."""

        diff_boll_to_goal = tool.get_distance(world_model.ball.pos, world_model.field_points.our_goal_center)
        diff_boll_to_robot = tool.get_distance(world_model.ball.pos, world_model.robots.our_robots[self.robot_id].pos)

        # レシーブフラグがTrueかつボールが動いている場合は、自分がレシーブできるかを判断する
        best_receive_score = world_model.robot_activity.our_ball_receive_score[0]
        if (
            self.do_receive
            and world_model.ball_activity.ball_is_moving
            and best_receive_score.robot_id == self.robot_id
            and best_receive_score.intercept_time != float("inf")
        ):
            # 自分がレシーブできる場合
            return self.tactic_receive.run(world_model)

        elif world_model.robot_activity.our_robots_by_ball_distance[0] == self.robot_id and (
            diff_boll_to_goal > self.diff_goal_threshold or diff_boll_to_robot < self.very_close_to_ball_threshold
        ):
            # ボールに一番近いかつ
            # ゴールからある程度遠い場合 or ボールに非常に近い場合はボールを操作する
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

        # シュート成功率が一番高いところに向かってパスする(クリア)
        self.tactic_pass.target_pos = world_model.kick_target.best_shoot_target.pos
        return self.tactic_pass.run(world_model)
