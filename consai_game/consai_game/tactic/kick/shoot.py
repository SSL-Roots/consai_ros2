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

"""シュート動作に関するTacticを定義するモジュール."""

import numpy as np

from consai_msgs.msg import MotionCommand, State2D

from consai_tools.geometry import geometry_tools as tool

from consai_game.world_model.world_model import WorldModel
from consai_game.core.tactic.tactic_base import TacticBase, TacticState

from transitions import Machine


class ShootStateMachine(Machine):
    """シュート状態遷移マシン."""

    BALL_NEAR_THRESHOLD = 0.5  # ボールが近いとみなす距離の閾値[m]
    SHOOT_ANGLE_THRESHOLD = 5  # シュート角度の閾値[degree]

    def __init__(self, name):
        """シュート状態遷移マシンのインスタンスを初期化する関数."""
        self.name = name

        # 状態定義
        states = ["chasing", "aiming", "shooting"]

        # 遷移定義
        transitions = [
            {"trigger": "ball_near", "source": "chasing", "dest": "aiming"},
            {"trigger": "ball_far", "source": "aiming", "dest": "chasing"},
            {"trigger": "shoot", "source": "aiming", "dest": "shooting"},
            {"trigger": "reaiming", "source": "shooting", "dest": "aiming"},
            {"trigger": "done_shooting", "source": "shooting", "dest": "chasing"},
            {"trigger": "reset", "source": "*", "dest": "chasing"},
        ]

        # ステートマシン構築
        super().__init__(model=self, states=states, transitions=transitions, initial="chasing")

    def update(self, dist_to_ball: float, shoot_diff_angle: float):
        """状態遷移を更新する関数."""
        if self.state == "chasing" and dist_to_ball <= self.BALL_NEAR_THRESHOLD:
            self.ball_near()

        elif self.state == "aiming" and dist_to_ball > self.BALL_NEAR_THRESHOLD:
            self.ball_far()

        elif self.state == "aiming" and shoot_diff_angle < self.SHOOT_ANGLE_THRESHOLD:
            self.shoot()

        elif self.state == "shooting" and shoot_diff_angle < self.SHOOT_ANGLE_THRESHOLD:
            self.reaiming()

        elif self.state == "shooting" and shoot_diff_angle >= self.SHOOT_ANGLE_THRESHOLD:
            self.done_shooting()


class Shoot(TacticBase):
    """指定した位置にシュートするTactic."""

    KICK_POWER_OFF = 0.0
    KICK_POWER_ON = 5.0
    CHASING_BALL_APPROACH_X = 0.5

    def __init__(self):
        """シュートのTacticインスタンスを初期化する関数."""
        super().__init__()

        self.move_pos = State2D()

        self.machine = ShootStateMachine("robot")

    def reset(self, robot_id: int) -> None:
        """ロボットIDを設定し, Tacticの状態をRUNNINGにリセットする関数."""
        self.robot_id = robot_id
        self.state = TacticState.RUNNING
        self.machine.reset()

    def run(self, world_model: WorldModel) -> MotionCommand:
        """シュートを実行するためのMotionCommandを生成する関数."""
        command = MotionCommand()
        command.robot_id = self.robot_id
        command.mode = MotionCommand.MODE_NAVI

        # キックターゲットを取得
        kick_target_model = world_model.kick_target
        if kick_target_model.best_shoot_target.success_rate > 50:
            target_pos = kick_target_model.best_shoot_target.pos
        else:
            target_pos = State2D()
            target_pos.x = 6.0
            target_pos.y = 0.0

        # ボールの位置を取得
        ball_pos = world_model.ball.pos
        robot_pos = world_model.robots.our_robots.get(self.robot_id).pos

        # ロボットとボールの距離を計算
        dist_to_ball = tool.get_distance(ball_pos, robot_pos)

        # シュートの角度を計算
        shoot_angle = tool.get_angle(ball_pos, target_pos)
        shoot_diff_angle = abs(tool.angle_normalize(robot_pos.theta - shoot_angle))

        self.machine.update(dist_to_ball, np.rad2deg(shoot_diff_angle))

        if self.machine.state == "chasing":
            command.kick_power = self.KICK_POWER_OFF
            self.move_pos.x = ball_pos.x - self.CHASING_BALL_APPROACH_X
            self.move_pos.y = ball_pos.y
            self.move_pos.theta = tool.get_angle(robot_pos, ball_pos)

        elif self.machine.state == "aiming":
            # 蹴る方向に向けて移動
            self.move_pos.x = ball_pos.x - 0.1 * np.cos(shoot_angle)
            self.move_pos.y = ball_pos.y - 0.1 * np.sin(shoot_angle)
            self.move_pos.theta = shoot_angle

        elif self.machine.state == "shooting":
            # シュートの角度が適切な場合、シュートを実行
            self.move_pos.theta = shoot_angle
            command.kick_power = self.KICK_POWER_ON

        command.desired_pose = self.move_pos

        return command
