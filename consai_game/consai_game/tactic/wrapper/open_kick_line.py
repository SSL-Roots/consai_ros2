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

from consai_tools.geometry import geometry_tools as tool


class OpenKickLine(TacticBase):
    """キックラインを開くWrapperTactic.

    OpenKickLine(tactic=Position()) のように使用する
    """

    AVOID_RADIUS = 0.3  # ボールを避ける半径 [m]

    def __init__(self, tactic=TacticBase):
        """inner_tacticを初期化する関数."""
        super().__init__()
        self.inner_tactic = tactic

    def reset(self, robot_id: int) -> None:
        """inner_tacticをリセットする関数."""
        super().reset(robot_id)
        self.inner_tactic.reset(robot_id)

    def run(self, world_model: WorldModel) -> MotionCommand:
        """キックラインを開くようにdesired_poseを上書きする."""
        command = self.inner_tactic.run(world_model)

        # キックラインを開く処理を追加
        # ここでは、desired_poseを適切な位置に変更するロジックを実装する
        shoot_target_pos = world_model.kick_target.best_shoot_target.pos
        pass_target = world_model.kick_target.best_pass_target
        ball_pos = world_model.ball.pos

        # 目標位置がシュートラインに干渉しているか計算
        is_shoot_line_blocked = tool.is_on_line(
            pose=command.desired_pose, line_pose1=ball_pos, line_pose2=shoot_target_pos, tolerance=self.AVOID_RADIUS
        )

        # 目標位置がパスラインに干渉しているか計算
        is_pass_line_blocked = tool.is_on_line(
            pose=command.desired_pose,
            line_pose1=ball_pos,
            line_pose2=pass_target.robot_pos,
            tolerance=self.AVOID_RADIUS,
        )

        # パスを受けるロボットが自分でなく、シュートラインもしくはパスラインに干渉している場合
        if self.robot_id != pass_target.robot_id and (is_shoot_line_blocked or is_pass_line_blocked):
            # パスラインを開く
            command.navi_options.avoid_ball = True
            command.desired_pose.y = command.desired_pose.y + self.AVOID_RADIUS
            # stateを上書きする
            self.state = self.inner_tactic.state

        return command
