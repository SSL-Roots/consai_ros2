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


from consai_game.world_model.world_model import WorldModel
from consai_game.core.tactic.tactic_base import TacticBase, TacticState
from consai_msgs.msg import MotionCommand


class Position(TacticBase):
    """指定した位置に移動するTactic."""

    def __init__(self, x=0.0, y=0.0, theta=0.0):
        super().__init__()
        self.x = x
        self.y = y
        self.theta = theta

    def reset(self, robot_id: int) -> None:
        self.robot_id = robot_id
        self.state = TacticState.RUNNING

    def run(self, world_model: WorldModel) -> MotionCommand:
        command = MotionCommand()
        command.robot_id = self.robot_id
        command.mode = MotionCommand.MODE_NAVI
        command.desired_pose.x = self.x
        command.desired_pose.y = self.y
        command.desired_pose.theta = self.theta

        return command
