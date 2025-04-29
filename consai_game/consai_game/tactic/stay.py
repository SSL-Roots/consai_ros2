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

"""ロボットに制御をかけたまま停止させるTacticを定義するモジュール."""

from consai_msgs.msg import MotionCommand

from consai_game.world_model.world_model import WorldModel
from consai_game.core.tactic.tactic_base import TacticBase


class Stay(TacticBase):
    """ロボットに制御をかけたまま停止させるTactic.

    Stopと違い、動作目標位置を生成するため、進入禁止エリア回避を行える.
    """

    def __init__(self):
        super().__init__()

    def run(self, world_model: WorldModel) -> MotionCommand:
        """その場にとどまるMotionCommandを返す."""
        robot_pos = world_model.robots.our_visible_robots[self.robot_id].pos

        command = MotionCommand()
        command.robot_id = self.robot_id
        command.mode = MotionCommand.MODE_NAVI
        # 目標位置をロボット自身の座標にして、その場にとどまる
        command.desired_pose = robot_pos

        return command
