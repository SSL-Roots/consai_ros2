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

"""ロボットを停止させるTacticを定義するモジュール."""

from consai_msgs.msg import MotionCommand

from consai_game.world_model.world_model import WorldModel
from consai_game.core.tactic.tactic_base import TacticBase


class Stop(TacticBase):
    """ロボットを停止させるTactic."""

    def __init__(self):
        """StopTacticのインスタンスを初期化する関数."""
        super().__init__()

    def run(self, world_model: WorldModel) -> MotionCommand:
        """ロボットを停止させるためのMotionCommandを生成する関数."""
        command = MotionCommand()
        command.robot_id = self.robot_id
        command.mode = MotionCommand.MODE_DIRECT_VELOCITY
        command.desired_velocity.x = 0.0
        command.desired_velocity.y = 0.0
        command.desired_velocity.theta = 0.0

        return command
