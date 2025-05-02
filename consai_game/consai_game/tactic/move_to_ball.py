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

from consai_game.core.tactic.tactic_base import TacticBase
from consai_game.world_model.world_model import WorldModel

from consai_msgs.msg import MotionCommand
from consai_msgs.msg import State2D

from consai_tools.geometry import geometry_tools as tool


class MoveToBall(TacticBase):
    """ボールに近づくTactic.

    ロボットの現在位置からまっすぐボールに近づく.
    距離を調整すれば、ボールから離れる動作としても使用可能.
    """

    def __init__(self, distance=0.5):
        super().__init__()

        # ボールとの距離
        self.distance = distance

    def run(self, world_model: WorldModel) -> MotionCommand:
        """ロボットからボールへ直線を引き、その線上でボールからdistanceだけ離れるコマンドを生成する."""
        command = MotionCommand()
        command.robot_id = self.robot_id
        command.mode = MotionCommand.MODE_NAVI

        # ボールの位置を取得
        ball_pos = world_model.ball.pos
        # ロボットの位置を取得
        robot_pos = world_model.robots.our_robots.get(self.robot_id).pos

        # ボールを中心に、ロボットを+X軸とする座標系を作る
        trans = tool.Trans(ball_pos, tool.get_angle(ball_pos, robot_pos))

        command.desired_pose = trans.inverted_transform(State2D(x=self.distance, y=0.0))
        command.desired_pose.theta = tool.get_angle(robot_pos, ball_pos)  # ボールの方向を向く

        return command
