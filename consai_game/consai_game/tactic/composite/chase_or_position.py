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
ボールに一番近ければボールを取りに行く、そうでなければ指定した位置に移動するモジュール.

PositionとChaseBallのTacticを組み合わせたTactic
"""

from consai_game.core.tactic.tactic_base import TacticBase
from consai_game.tactic.position import Position
from consai_game.tactic.chase_ball import ChaseBall
from consai_game.world_model.world_model import WorldModel

from consai_msgs.msg import MotionCommand


class ChaseOrPosition(TacticBase):
    """
    ボールに一番近ければボールを取りに行く、そうでなければ指定した位置に移動する.

    ChaseBallとPositionを利用する.
    """

    def __init__(self, x=0.0, y=0.0, theta=0.0):
        """Initialize the DefendGoal tactic."""
        super().__init__()
        self.x = x
        self.y = y
        self.theta = theta

        self.tactic_chase = ChaseBall()
        self.tactic_position = Position(x, y, theta)

    def reset(self, robot_id: int) -> None:
        """Reset the tactic state for the specified robot."""
        super().reset(robot_id)

        # 所有するTacticも初期化する
        self.tactic_chase.reset(robot_id)
        self.tactic_position.reset(robot_id)

    def exit(self):
        super().exit()

        # 所有するTacticもexitする
        self.tactic_chase.exit()
        self.tactic_position.exit()

    def run(self, world_model: WorldModel) -> MotionCommand:
        """Run the tactic and return a MotionCommand based on the ball's position and movement."""
        if world_model.robot_activity.our_robots_by_ball_distance[0] == self.robot_id:
            # ボールに近い場合はボールを追いかける
            command = self.tactic_chase.run(world_model)
        else:
            # ボールに近くない場合は指定した位置に移動する
            command = self.tactic_position.run(world_model)

        return command
