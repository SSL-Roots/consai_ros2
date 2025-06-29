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

from consai_game.core.tactic.composite_tactic_base import CompositeTacticBase
from consai_game.tactic.position import Position
from consai_game.tactic.chase_ball import ChaseBall
from consai_game.world_model.world_model import WorldModel

from consai_msgs.msg import MotionCommand


class ChaseOrPosition(CompositeTacticBase):
    """
    ボールに一番近ければボールを取りに行く、そうでなければ指定した位置に移動する.

    ChaseBallとPositionを利用する.
    """

    def __init__(self, x=0.0, y=0.0, theta=0.0):
        """Initialize the DefendGoal tactic."""
        super().__init__(tactic_chase=ChaseBall(), tactic_position=Position(x, y, theta))

        self.x = x
        self.y = y
        self.theta = theta

    def run(self, world_model: WorldModel) -> MotionCommand:
        """Run the tactic and return a MotionCommand based on the ball's position and movement."""
        if world_model.robot_activity.our_robots_by_ball_distance[0] == self.robot_id:
            # ボールに近い場合はボールを追いかける
            return self.run_sub_tactic(self.tactic_chase, world_model)
        else:
            # ボールに近くない場合は指定した位置に移動する
            return self.run_sub_tactic(self.tactic_position, world_model)
