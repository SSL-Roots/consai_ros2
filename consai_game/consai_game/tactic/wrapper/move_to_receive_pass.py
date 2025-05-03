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

    def exit(self):
        """inner_tacticをexitする関数."""
        super().exit()
        self.inner_tactic.exit()

    def run(self, world_model: WorldModel) -> MotionCommand:
        """パスラインを開くようにdesired_poseを上書きする."""
        command = self.inner_tactic.run(world_model)

        ball_pos = world_model.ball.pos
        their_robot = world_model.robots.their_visible_robots

        # ボールを持っていないロボットとボールを結んだtransを作成
        trans = tool.Trans(ball_pos, tool.get_angle(ball_pos, command.desired_pose))
        # 相手ロボットの位置がパスラインの上下どちらに多く干渉しているかを確認
        upper_count = 0
        lower_count = 0

        tr_desired_pose = trans.transform(command.desired_pose)

        for their in their_robot.values():
            tr_their_pos = trans.transform(their.pos)
            # 相手ロボットの位置がボールからパスを受けるロボットの1m後ろまでの範囲にいるか
            if tr_their_pos.x > 0 and tr_their_pos.x < tr_desired_pose.x + 1.0:
                # 相手ロボットの位置がパスラインの上下どちらにいるか
                if 0 < tr_their_pos.y and tr_their_pos.y < self.THRESHOULD_RADIUS:
                    upper_count += 1
                elif 0 > tr_their_pos.y and tr_their_pos.y > -self.THRESHOULD_RADIUS:
                    lower_count += 1

        # パスラインに干渉している相手ロボットが少ない方向に移動する
        if upper_count == 0 and lower_count == 0:
            # 防いでいる相手ロボットがいなければ現在の位置にとどまる
            pass
        else:
            # パスラインの上下のどちらにロボットが多くいるかで移動先を指定
            if upper_count > lower_count:
                tr_desired_pose.y -= self.THRESHOULD_RADIUS * 2
            else:
                tr_desired_pose.y += self.THRESHOULD_RADIUS * 2

            # 上書きするdesired_pose.yがフィールド外に出る場合内側に、ディフェンスエリアに入る場合は目標位置を反対側に設定して回避するように設定
            if trans.inverted_transform(tr_desired_pose).y > world_model.field.half_width or (
                trans.inverted_transform(tr_desired_pose).x < world_model.field.penalty_depth
                and -world_model.field.half_penalty_width < trans.inverted_transform(tr_desired_pose).y
                and trans.inverted_transform(tr_desired_pose).y < world_model.field.half_penalty_width
            ):
                tr_desired_pose.y -= self.THRESHOULD_RADIUS * 4
            elif trans.inverted_transform(tr_desired_pose).y < -world_model.field.half_width or (
                trans.inverted_transform(tr_desired_pose).x < world_model.field.penalty_depth
                and -world_model.field.half_penalty_width < trans.inverted_transform(tr_desired_pose).y
                and trans.inverted_transform(tr_desired_pose).y < world_model.field.half_penalty_width
            ):
                tr_desired_pose.y += self.THRESHOULD_RADIUS * 4
            new_desired_pose = trans.inverted_transform(tr_desired_pose)

            # desired_poseを上書きする
            command.desired_pose.x = new_desired_pose.x
            command.desired_pose.y = new_desired_pose.y
        # ロボットがボールを向くようにthetaを上書きする
        command.desired_pose.theta = tool.get_angle(command.desired_pose, ball_pos)
        # stateを上書きする
        self.state = self.inner_tactic.state
        return command
