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
BallApproach Tactic.

ボールアプローチのTactic.
"""

from consai_game.core.tactic.tactic_base import TacticBase
from consai_game.world_model.world_model import WorldModel

from consai_msgs.msg import MotionCommand
from consai_msgs.msg import State2D

from consai_tools.geometry import geometry_tools as tool

import numpy as np

from transitions.extensions import GraphMachine


class BallApproachStateMachine(GraphMachine):
    """ドリブルの状態遷移マシン."""

    # ボールが近いとみなす距離の閾値[m]
    BALL_NEAR_THRESHOLD = 0.4
    # ボールを保持していると判定する距離の閾値[m]
    BALL_GET_THRESHOLD = 0.2
    # 目的地が近いとみなす距離の閾値[m]
    DIST_TARGET_TO_BALL_THRESHOLD = 0.1

    # ドリブル角度の閾値[degree]
    DRIBBLE_ANGLE_THRESHOLD = 5

    def __init__(self, name):
        """状態遷移の初期化"""
        self.name = name

        # 状態定義
        states = ["arrived", "chasing", "aiming"]

        # 遷移定義
        transitions = [
            {"trigger": "chase", "source": "arrived", "dest": "chasing"},
            {"trigger": "aim", "source": "chasing", "dest": "aiming"},
            {"trigger": "arrival", "source": "aiming", "dest": "arrived"},
            {"trigger": "reset", "source": "*", "dest": "arrived"},
        ]

        # ステートマシン構築
        super().__init__(
            model=self,
            states=states,
            transitions=transitions,
            initial="arrived",
            auto_transitions=True,
            ordered_transitions=False,
            title="",
            show_auto_transitions=False,
        )

    def update(self, ball_trans_pos: float, robot_trans_pos: float):
        """状態遷移"""
        dist_ball_to_target = tool.get_distance(ball_trans_pos, robot_trans_pos)
        angle_ball_to_target = tool.angle_normalize(tool.get_angle(ball_trans_pos, robot_trans_pos))

        if self.state == "arrived" and self.BALL_GET_THRESHOLD < dist_ball_to_target:
            self.chase()
        if self.state == "chasing" and ball_trans_pos.x < robot_trans_pos.x and angle_ball_to_target < 5:
            self.aim()
        elif (
            self.state == "aiming"
            and dist_ball_to_target < self.BALL_GET_THRESHOLD
            and ball_trans_pos.x < robot_trans_pos.x
        ):
            self.arrival()


class BallApproach(TacticBase):
    """ボールアプローチのTactic"""

    # ボールアプローチ時の距離[m]
    BALL_APPROACH_DIST = 0.3
    # アプローチ時に加算する速度のゲイン[s]
    VEL_GAIN = 1.0
    # ロボット+ボールの距離[m]
    ROBOT_TO_BALL_DIST = 0.15

    # ボール追跡時の回り込みの距離[m]
    # CHASING_BALL_APPROACH_DIST = 0.5

    def __init__(self, x: float = 0.0, y: float = 0.0):
        """Initialize the DefendGoal tactic"""
        super().__init__()
        self.move_pos = State2D()
        self.machine = BallApproachStateMachine("robot")

        self.target_pos = State2D(x=x, y=y)

    def reset(self, robot_id: int) -> None:
        """Reset the tactic state for the specified robot"""
        self.robot_id = robot_id
        self.machine.reset()

    def run(self, world_model: WorldModel) -> MotionCommand:  # , approach_angle: float
        """Run the tactic and return a MotionCommand based on the ball's position and movement"""
        command = MotionCommand()
        command.robot_id = self.robot_id
        command.mode = MotionCommand.MODE_NAVI

        # 基本はボール回避をしない
        command.navi_options.avoid_ball = False

        # ボールの位置を取得
        ball_pos = world_model.ball.pos
        # ボールの予測位置を取得
        next_ball_pos = world_model.ball_activity.next_ball_pos
        # ボールの移動量を取得
        ball_movement = world_model.ball_activity.ball_movement

        # ロボットの位置を取得
        robot_pos = world_model.robots.our_robots.get(self.robot_id).pos

        # 目標位置とボール間の角度
        angle_target_to_ball = tool.get_angle(self.target_pos, ball_pos)

        # 目標位置とボール予測位置の角度
        angle_target_to_next_ball = tool.get_angle(self.target_pos, next_ball_pos)
        trans = tool.Trans(next_ball_pos, angle_target_to_next_ball)
        ball_trans_pos = trans.transform(ball_pos)
        robot_trans_pos = trans.transform(robot_pos)

        # 状態遷移を更新
        self.machine.update(ball_trans_pos, robot_trans_pos)

        if self.machine.state == "chasing":
            # ボールを追いかける
            self.move_pos.x = ball_pos.x + self.BALL_APPROACH_DIST * np.cos(angle_target_to_next_ball)
            self.move_pos.y = ball_pos.y + self.BALL_APPROACH_DIST * np.sin(angle_target_to_next_ball)

            # 速度方向に位置をずらす
            self.move_pos.x += ball_movement.x
            self.move_pos.y += ball_movement.y

            # 追いかけるときはボール回避を実行
            command.navi_options.avoid_ball = True
        elif self.machine.state == "aiming":
            # ボールの近くへ移動
            self.move_pos.x = ball_pos.x + self.ROBOT_TO_BALL_DIST * np.cos(angle_target_to_next_ball)
            self.move_pos.y = ball_pos.y + self.ROBOT_TO_BALL_DIST * np.sin(angle_target_to_next_ball)

            # 速度方向に位置をずらす
            self.move_pos.x += ball_movement.x
            self.move_pos.y += ball_movement.y
        elif self.machine.state == "arrived":
            # ボールの近くへ移動
            self.move_pos.x = ball_pos.x + self.ROBOT_TO_BALL_DIST * np.cos(angle_target_to_ball)
            self.move_pos.y = ball_pos.y + self.ROBOT_TO_BALL_DIST * np.sin(angle_target_to_ball)

        self.move_pos.theta = angle_target_to_next_ball + np.pi
        command.desired_pose = self.move_pos

        return command
