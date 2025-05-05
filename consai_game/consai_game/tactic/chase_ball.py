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

    def __init__(self, chase_distance: float = 0.8):
        """Initialize the DefendGoal tactic."""
        super().__init__()
        self.move_pos = State2D()
        self.chase_distance = chase_distance

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
        # 味方ゴールの中心座標
        our_goal_pos = State2D(x=-world_model.field.half_length, y=0.0)

        # ロボットとボール間の角度を計算
        angle_robot_to_ball = tool.get_angle(robot_pos, ball_pos)
        # ボールと味方ゴール間の角度を計算
        angle_to_ball_our_goal = tool.get_angle(ball_pos, our_goal_pos)

        trans = tool.Trans(ball_pos, angle_to_ball_our_goal)
        tr_ball_pos = trans.transform(ball_pos)
        tr_robot_pos = State2D(x=tr_ball_pos.x + self.chase_distance, y=0.0)
        self.move_pos = trans.inverted_transform(tr_robot_pos)
        self.move_pos.theta = angle_robot_to_ball

        command.desired_pose = self.move_pos

        return command
