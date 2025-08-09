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
from consai_game.tactic.receive import Receive
from consai_game.world_model.world_model import WorldModel

from consai_msgs.msg import MotionCommand

from consai_tools.geometry import geometry_tools as tool


class DefendGoal(TacticBase):
    """自ゴールを守るTactic."""

    def __init__(self):
        """Initialize the DefendGoal tactic."""
        super().__init__()
        self.receive = Receive()

    def reset(self, robot_id: int) -> None:
        """Reset the tactic state for the specified robot."""
        super().reset(robot_id)
        self.receive.reset(robot_id)

    def run(self, world_model: WorldModel) -> MotionCommand:
        """Run the tactic and return a MotionCommand based on the ball's position and movement."""
        command = MotionCommand()
        command.robot_id = self.robot_id
        robot = world_model.robots

        # ボールの位置を取得
        ball_pos = world_model.ball.pos
        # ボールの速度を取得
        next_ball_pos = world_model.ball_activity.next_ball_pos

        # ボールの進行方向の直線に関する傾きと切片を計算
        slope, intercept, flag = tool.get_line_parameter(ball_pos, next_ball_pos)

        x = -world_model.field.half_length + robot.robot_radius
        y = slope * x + intercept

        if abs(y) < world_model.field.half_goal_width:
            # ゴールに入りそうな場合

            # ゴール前のボール進行方向上の位置を計算
            receive_command = self.receive.run(world_model, 90)

            if x < receive_command.desired_pose.x:
                # ディフェンス位置がゴールラインより前になる場合
                # ボールの垂直方向に移動
                x = receive_command.desired_pose.x
                y = receive_command.desired_pose.y

        y = max(min(y, world_model.field.half_goal_width), -world_model.field.half_goal_width)

        command.desired_pose.x = x
        command.desired_pose.y = y
        command.desired_pose.theta = 0.0

        return command
