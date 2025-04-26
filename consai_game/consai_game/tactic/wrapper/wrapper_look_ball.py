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

from consai_msgs.msg import MotionCommand

from consai_game.world_model.world_model import WorldModel
from consai_game.core.tactic.tactic_base import TacticBase, TacticState
from consai_tools.geometry import geometry_tools as tool


class WrapperLookBall(TacticBase):
    """ボールを見るようにthetaを上書きするWrapperTactic.

    WrapperLookBall(tactic=Position()) のように使用する
    """

    def __init__(self, tactic=TacticBase):
        """Positionのインスタンスを初期化する関数."""
        super().__init__()
        self.inner_tactic = tactic

    def reset(self, robot_id: int) -> None:
        """ロボットIDを設定し、Tacticの状態をRUNNINGにリセットする関数."""
        self.robot_id = robot_id
        self.state = TacticState.RUNNING
        self.inner_tactic.reset(robot_id)

    def run(self, world_model: WorldModel) -> MotionCommand:
        """ボールを見るようにdesired_poseを上書きする.`"""
        command = self.inner_tactic.run(world_model)

        ball_pos = world_model.ball.pos
        command.desired_pose.theta = tool.get_angle(command.desired_pose, ball_pos)

        return command
