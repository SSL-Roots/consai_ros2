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


class SlowSafePosition(TacticBase):
    """ボールを避けながら指定した位置に移動するTactic."""

    # 最大速度の制限値 [m/s]
    MAX_VELOCITY = 1.0

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

        # 目標位置を設定
        command.desired_pose.x = self.x
        command.desired_pose.y = self.y
        command.desired_pose.theta = self.theta

        # 最大速度を制限
        command.desired_velocity.x = self.MAX_VELOCITY
        command.desired_velocity.y = self.MAX_VELOCITY

        # NaviOptionsを設定してボールを避ける
        command.navi_options.avoid_ball = True
        command.navi_options.ball_avoid_radius = 0.5  # 500mm
        command.navi_options.avoid_our_robots = True
        command.navi_options.avoid_their_robots = True
        command.navi_options.avoid_pushing = True
        command.navi_options.avoid_defense_area = True
        command.navi_options.avoid_placement_area = True

        return command
