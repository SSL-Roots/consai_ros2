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
BallClear Tactic.

自ディフェンスエリアにあるボールをクリアする.
"""

from consai_game.core.tactic.tactic_base import TacticBase
from consai_game.tactic.kick.shoot import Shoot
from consai_game.world_model.world_model import WorldModel

from consai_msgs.msg import MotionCommand


class BallClear(TacticBase):
    """自ディフェンスエリアにあるボールをクリアするTactic."""

    def __init__(self):
        """Initialize the DefendGoal tactic."""
        super().__init__()
        self.shoot = Shoot()

    def reset(self, robot_id: int) -> None:
        """Reset the tactic state for the specified robot."""
        self.robot_id = robot_id
        self.shoot.reset(robot_id)

    def run(self, world_model: WorldModel) -> MotionCommand:
        """Run the tactic and return a MotionCommand based on the ball's position and movement."""
        command = MotionCommand()
        command.robot_id = self.robot_id

        command = self.shoot.run(world_model)

        return command
