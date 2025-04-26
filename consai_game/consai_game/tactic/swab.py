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

"""雑巾がけのTactic."""

from consai_msgs.msg import MotionCommand
from consai_msgs.msg import State2D

from consai_game.world_model.world_model import WorldModel
from consai_game.core.tactic.tactic_base import TacticBase, TacticState

from transitions import Machine

import math


class SwabStateMachine(Machine):
    """雑巾がけの状態遷移マシン."""

    def __init__(self, name):
        """状態遷移の初期化."""
        self.name = name

        # 状態定義
        states = ["moving_left", "moving_right"]

        # 遷移定義
        transitions = [
            {"trigger": "move_right", "source": "moving_left", "dest": "moving_right"},
            {"trigger": "move_left", "source": "moving_right", "dest": "moving_left"},
            {"trigger": "reset", "source": "*", "dest": "moving_left"},
        ]

        # ステートマシン構築
        super().__init__(model=self, states=states, transitions=transitions, initial="moving_left")

    def update(self, all_robots_arrived: float):
        """状態遷移."""
        if self.state == "moving_left" and all_robots_arrived:
            self.move_right()
        elif self.state == "moving_right" and all_robots_arrived:
            self.move_left()


class Swab(TacticBase):
    """指定した位置に移動するTactic."""

    def __init__(self):
        """Positionのインスタンスを初期化する関数."""
        super().__init__()
        self.target_pos = State2D()
        self.machine = SwabStateMachine("robot")

    def reset(self, robot_id: int) -> None:
        """ロボットIDを設定し、Tacticの状態をRUNNINGにリセットする関数."""
        self.robot_id = robot_id
        self.state = TacticState.RUNNING
        self.machine.reset()

    def run(self, world_model: WorldModel) -> MotionCommand:
        """指定した位置に移動するためのMotionCommandを生成する関数."""
        command = MotionCommand()
        command.robot_id = self.robot_id
        command.mode = MotionCommand.MODE_DIRECT_POSE

        # 状態遷移を更新
        self.machine.update(world_model.robot_activity.all_robots_arrived)

        # フィールドの大きさを取得
        # self.length = world_model.field.length
        self.width = world_model.field.width
        self.half_length = world_model.field.half_length
        self.half_width = world_model.field.half_width

        # 目標位置を設定
        self.target_pos.y = self.half_width * 0.9 - (self.width * 0.9) * self.robot_id / 11
        if self.machine.state == "moving_left":
            self.target_pos.x = -self.half_length * 0.9
            self.target_pos.theta = math.pi
        elif self.machine.state == "moving_right":
            self.target_pos.x = self.half_length * 0.9
            self.target_pos.theta = 0.0

        command.desired_pose.x = self.target_pos.x
        command.desired_pose.y = self.target_pos.y
        command.desired_pose.theta = self.target_pos.theta

        return command
