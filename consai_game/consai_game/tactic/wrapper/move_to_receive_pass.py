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


class MoveToReceivePass(TacticBase):
    """パスラインが相手ロボットとかぶっていたらパスをもらえる位置に移動するWrapperTactic.

    MoveToReceivePass(tactic=Position()) のように使用する
    """

    THRESHOULD_RADIUS = 0.5  # パスラインに対して相手ロボットが干渉するとみなす半径[m]

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

        ball_pos = world_model.ball.pos
        their_robot = world_model.robots.their_visible_robots

        # ボールを持っていないロボットとボールを結んだtransを作成
        trans = tool.Trans(command.desired_pose, tool.get_angle(command.desired_pose, ball_pos))
        # 相手ロボットの位置がパスライン上限おどちらに多く干渉しているかを確認
        upper_count = 0
        lower_count = 0
        for their in their_robot.values():
            their_pos_trans = trans.transform(their.pos)
            their_pos = trans.inverted_transform(their_pos_trans)
            if their_pos.x > 0 and 0 < their_pos.y and their_pos.y < self.THRESHOULD_RADIUS:
                upper_count += 1
            elif their_pos.x > 0 and 0 > their_pos.y and their_pos.y > -self.THRESHOULD_RADIUS:
                lower_count += 1

        # パスラインに干渉している相手ロボットが少ない方向に移動する
        desired_pose_trans = trans.transform(command.desired_pose)
        desired_pose_from_pass_line = trans.inverted_transform(desired_pose_trans)
        if upper_count == 0 and lower_count == 0:
            pass
        else:
            if upper_count > lower_count:
                command.desired_pose.y = desired_pose_from_pass_line.y - self.THRESHOULD_RADIUS
            else:
                command.desired_pose.y = desired_pose_from_pass_line.y + self.THRESHOULD_RADIUS

            # desired_pose.yがフィールド外に出る場合は内側に回避するように設定
            if command.desired_pose.y > world_model.field.half_width:
                command.desired_pose.y = desired_pose_from_pass_line.y - self.THRESHOULD_RADIUS * 2
            elif command.desired_pose.y < -world_model.field.half_width:
                command.desired_pose.y = desired_pose_from_pass_line.y + self.THRESHOULD_RADIUS * 2
        # stateを上書きする
        self.state = self.inner_tactic.state
        return command
