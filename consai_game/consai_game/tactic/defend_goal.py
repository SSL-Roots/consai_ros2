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
DefendGoal Tactic.

ボールの位置と動きに基づいて自チームのゴールを守るためのTacticを定義.
"""

from consai_game.core.tactic.tactic_base import TacticBase
from consai_game.core.tactic.tactic_base import TacticState
from consai_game.world_model.world_model import WorldModel

from consai_msgs.msg import MotionCommand

from consai_tools.geometry import geometry_tools as tool


class DefendGoal(TacticBase):
    """自ゴールを守るTactic."""

    def __init__(self):
        """Initialize the DefendGoal tactic."""
        super().__init__()

        # ディフェンス位置(x座標)を調整する変数
        self.margin_defend_x = 0.1
        # ボールが動いているか判定する閾値
        self.ball_move_threshold = 0.1

    def reset(self, robot_id: int) -> None:
        """Reset the tactic state for the specified robot."""
        self.robot_id = robot_id
        self.state = TacticState.RUNNING

    def run(self, world_model: WorldModel) -> MotionCommand:
        """Run the tactic and return a MotionCommand based on the ball's position and movement."""
        command = MotionCommand()
        command.robot_id = self.robot_id

        # ボールを回避をしない
        command.navi_options.avoid_ball = False
        # ディフェンスエリアを回避をしない
        command.navi_options.avoid_defense_area = False

        # ボールが自チームエリアにあるか判定結果を取得
        is_in_our_side = world_model.ball_position.is_in_our_side()
        # ボールが自チームディフェンスエリアにあるか判定結果を取得
        is_in_our_defense_area = world_model.ball_position.is_in_our_defense_area()

        # ボールの位置を取得
        ball_pos = world_model.ball.pos
        # ボールの速度を取得
        ball_vel = world_model.ball.vel
        # ボールの未来の予測位置を取得
        ball_next_pos = world_model.ball.next_pos

        if is_in_our_side and ball_vel.x < -self.ball_move_threshold:
            # ボールが自チームエリアにあり, ゴールへ向かってくる場合

            # ボールの進行方向の直線に関する傾きと切片を計算
            slope, intercept, flag = tool.get_line_parameter(ball_pos, ball_next_pos)

            # ゴール前のボール進行方向上の位置を計算
            x = -world_model.field.half_length + self.margin_defend_x
            y = slope * x + intercept
        elif is_in_our_defense_area:
            # ボールが自ディフェンスエリアにある場合

            # y座標をボールと同じ位置にする
            x = -world_model.field.half_length + self.margin_defend_x
            y = ball_pos.y

            # Stateの終了
            self.state = TacticState.FINISHED
        else:
            # ボールがゴールへ向かって来ない場合

            # y座標をボールと同じ位置にする
            x = -world_model.field.half_length + self.margin_defend_x
            y = ball_pos.y

        if world_model.field.half_goal_width < abs(y):
            # 生成した位置がゴール端を超える場合

            # ゴール端になるようclamp
            y = max(min(y, world_model.field.half_goal_width), -world_model.field.half_goal_width)

        command.desired_pose.x = x
        command.desired_pose.y = y
        command.desired_pose.theta = 0.0

        return command
