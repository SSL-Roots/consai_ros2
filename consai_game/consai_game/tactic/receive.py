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
Receive Tactic.

ボールの軌道上でボールを待って受け取るTacticを定義.
"""

from consai_game.core.tactic.tactic_base import TacticBase
from consai_game.core.tactic.tactic_base import TacticState
from consai_game.world_model.world_model import WorldModel

from consai_msgs.msg import MotionCommand
from consai_msgs.msg import State2D

from consai_tools.geometry import geometry_tools as tool

import numpy as np


class Receive(TacticBase):
    """ボールの軌道上でボールを待って受け取るTactic."""

    # ドリブルON時の出力
    DRIBBLE_ON = 1.0
    # ドリブルOFF時の出力(0.0)
    DRIBBLE_OFF = 0.0

    def __init__(self):
        """Initialize the DefendGoal tactic."""
        super().__init__()
        self.receive_pos = State2D()

    def reset(self, robot_id: int) -> None:
        """Reset the tactic state for the specified robot."""
        self.robot_id = robot_id
        self.state = TacticState.RUNNING

    def run(self, world_model: WorldModel, diff_angle_threshold: int = 20) -> MotionCommand:
        """Run the tactic and return a MotionCommand based on the ball's position and movement."""
        command = MotionCommand()
        command.robot_id = self.robot_id

        # ボールを回避をしない
        command.navi_options.avoid_ball = False
        # ドリブラーOFF
        command.dribble_power = self.DRIBBLE_OFF

        # ロボットの位置を取得
        robot_pos = world_model.robots.our_robots.get(self.robot_id).pos

        # ボールの位置を取得
        ball_pos = world_model.ball.pos
        # ボールの未来の予測位置を取得
        next_ball_pos = world_model.ball_activity.next_ball_pos
        # ボールが動いているかのフラグ
        ball_is_moving = world_model.ball_activity.ball_is_moving

        # ロボットとボール間の角度を計算
        theta_robot_to_ball = tool.get_angle(robot_pos, ball_pos)
        # ボールと次のボール位置間の角度を計算
        theta_ball_to_next_ball = tool.get_angle(ball_pos, next_ball_pos)

        # 角度の差分を計算
        diff_theta = tool.angle_normalize(theta_robot_to_ball - theta_ball_to_next_ball - np.pi)
        diff_theta = np.rad2deg(abs(diff_theta))

        # 受取の座標
        self.receive_pos = robot_pos
        if ball_is_moving and diff_theta < diff_angle_threshold:
            # ボールが動いていてロボットに向かってくる場合

            # ボールの進行方向に対して垂直な位置を算出
            trans = tool.Trans(ball_pos, theta_ball_to_next_ball)
            receive_trans_pos = trans.transform(self.receive_pos)
            receive_trans_pos.y = 0.0
            self.receive_pos = trans.inverted_transform(receive_trans_pos)

        command.desired_pose = self.receive_pos
        command.desired_pose.theta = theta_robot_to_ball

        return command
