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
from consai_game.world_model.world_model import WorldModel

from consai_msgs.msg import MotionCommand


class GoaliePositioning(TacticBase):
    """ゴーリーのポジションを生成するTactic"""

    # ロボットの半径[m]
    ROBOT_RADIUS = 0.1

    def __init__(self):
        """Initialize the DefendGoal tactic."""
        super().__init__()

    def reset(self, robot_id: int) -> None:
        """Reset the tactic state for the specified robot."""
        super().reset(robot_id)

    def run(self, world_model: WorldModel) -> MotionCommand:
        """Run the tactic and return a MotionCommand based on the ball's position and movement."""
        command = MotionCommand()
        command.robot_id = self.robot_id

        # ボールの位置を取得
        ball_pos = world_model.ball.pos

        # x座標をゴール前にする
        x = -world_model.field.half_length + self.ROBOT_RADIUS
        # y座標をボールと同じ位置にする
        y = ball_pos.y

        # ゴールからはみ出ないようにclamp
        y = max(min(y, world_model.field.half_goal_width), -world_model.field.half_goal_width)

        command.desired_pose.x = x
        command.desired_pose.y = y
        command.desired_pose.theta = 0.0

        return command
