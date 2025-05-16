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

"""円軌道で移動するTacticを定義するモジュール."""

from consai_msgs.msg import MotionCommand

from consai_game.world_model.world_model import WorldModel
from consai_game.core.tactic.tactic_base import TacticBase

import numpy


class CircularMove(TacticBase):
    """円軌道で移動するTactic."""

    def __init__(self, radius=0.0, seconds=10, cw=True):
        """インスタンスを初期化する関数."""
        super().__init__()
        self.radius = radius
        self.counter = 0
        self.seconds = seconds
        self.cw = cw

    def reset(self, robot_id: int) -> None:
        """Reset the tactic state for the specified robot."""
        super().reset(robot_id)

    def exit(self):
        super().exit()

    def run(self, world_model: WorldModel, x=0.0, y=0.0) -> MotionCommand:
        """円軌道上の目標位置に移動するためのMotionCommandを生成する関数."""
        command = MotionCommand()
        command.robot_id = self.robot_id
        command.mode = MotionCommand.MODE_NAVI

        # 1周に必要なステップ数
        total_count = world_model.meta.update_rate * self.seconds

        # 現在のステップに応じて角度を算出
        dtheta = numpy.deg2rad(360) * self.counter / total_count
        self.counter += 1

        # 一周したらカウンタをリセット
        if self.counter == total_count:
            self.counter = 0

        # 回転方向の決定
        if self.cw is True:
            dtheta = -dtheta

        # 円軌道上の次の目標位置を計算
        command.desired_pose.x = numpy.cos(dtheta) * self.radius + x
        command.desired_pose.y = numpy.sin(dtheta) * self.radius + y

        command.navi_options.avoid_our_robots = False
        command.navi_options.avoid_pushing = False

        return command
