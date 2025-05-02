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
ChaseBall Tactic.

ボールを追いかけるTactic.
"""

from consai_game.core.tactic.tactic_base import TacticBase
from consai_game.world_model.world_model import WorldModel

from consai_msgs.msg import MotionCommand
from consai_msgs.msg import State2D

from consai_tools.geometry import geometry_tools as tool


class ChaseBall(TacticBase):
    """ボールを追いかけるTactic."""

    def __init__(self, margin_x=0.5, margin_y=0.0):
        """Initialize the DefendGoal tactic."""
        super().__init__()

        self.move_pos = State2D()
        # ボール追跡時にボールから離れる距離[m]
        self.margin_x = margin_x
        self.margin_y = margin_y

    def reset(self, robot_id: int) -> None:
        """Reset the tactic state for the specified robot."""
        self.robot_id = robot_id

    def run(self, world_model: WorldModel) -> MotionCommand:
        """Run the tactic and return a MotionCommand based on the ball's position and movement."""
        command = MotionCommand()
        command.robot_id = self.robot_id
        command.mode = MotionCommand.MODE_NAVI

        # ボールの位置を取得
        ball_pos = world_model.ball.pos
        # ロボットの位置を取得
        robot_pos = world_model.robots.our_robots.get(self.robot_id).pos

        # ロボットとボール間の角度を計算
        robot_to_ball_angle = tool.get_angle(robot_pos, ball_pos)

        self.move_pos.x = ball_pos.x - self.margin_x
        self.move_pos.y = ball_pos.y - self.margin_y
        self.move_pos.theta = robot_to_ball_angle

        command.desired_pose = self.move_pos

        return command
