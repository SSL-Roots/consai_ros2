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

"""キック動作に関するTacticを定義するモジュール."""

import numpy as np

from consai_msgs.msg import MotionCommand, State2D

from consai_tools.geometry import geometry_tools as tool

from consai_game.world_model.world_model import WorldModel
from consai_game.core.tactic.tactic_base import TacticBase, TacticState

from transitions import Machine


class KickStateMachine(Machine):
    """状態遷移マシン."""

    BALL_NEAR_THRESHOLD = 0.5  # ボールが近いとみなす距離の閾値[m]
    KICK_ANGLE_THRESHOLD = 5  # シュート角度の閾値[degree]
    BALL_KICK_THRESHOLD = 0.2  # ボールが蹴られたとみなす距離[m]

    def __init__(self, name):
        """状態遷移マシンのインスタンスを初期化する関数."""
        self.name = name

        # 状態定義
        states = ["chasing", "aiming", "kicking"]

        # 遷移定義
        transitions = [
            {"trigger": "ball_near", "source": "chasing", "dest": "aiming"},
            {"trigger": "ball_far", "source": "aiming", "dest": "chasing"},
            {"trigger": "kick", "source": "aiming", "dest": "kicking"},
            {"trigger": "reaiming", "source": "kicking", "dest": "aiming"},
            {"trigger": "done_kicking", "source": "kicking", "dest": "chasing"},
            {"trigger": "reset", "source": "*", "dest": "chasing"},
        ]

        # ステートマシン構築
        super().__init__(model=self, states=states, transitions=transitions, initial="chasing")

    def update(self, dist_to_ball: float, kick_diff_angle: float):
        """状態遷移を更新する関数."""
        if self.state == "chasing" and dist_to_ball < self.BALL_NEAR_THRESHOLD and kick_diff_angle < 90:
            self.ball_near()

        elif self.state == "aiming" and dist_to_ball > self.BALL_NEAR_THRESHOLD:
            self.ball_far()

        elif self.state == "aiming" and kick_diff_angle < self.KICK_ANGLE_THRESHOLD:
            self.kick()

        elif self.state == "kicking" and kick_diff_angle > self.KICK_ANGLE_THRESHOLD:
            self.reaiming()

        elif self.state == "kicking" and dist_to_ball > self.BALL_KICK_THRESHOLD:
            self.done_kicking()


class Kick(TacticBase):
    """指定した位置にボールを蹴るTactic."""

    MAX_KICK_POWER = 6.0  # 6.5 m/sを越えてはいけない
    CHASING_BALL_APPROACH = 0.2
    # 1m 以上ボールを持って移動すると、Excessive Dribbling違反になる
    TAPPING_KICK_POWER = 2.0  # ボールをコツコツ蹴ってドリブルするためのキックパワー

    def __init__(self, x=0.0, y=0.0, is_pass=False, is_tapping=False):
        """
        コンストラクタでキック目標位置と、キックの種類を設定する.

        Args:
            x (float) : キック目標位置のx座標
            y (float) : キック目標位置のy座標
            is_pass (bool) : パスをする場合はTrue
            is_tapping (bool) : ボールをコツコツ蹴って前進する場合はTrue

        """
        super().__init__()

        self.target_pos = State2D(x=x, y=y)
        self.is_pass = is_pass
        self.is_tapping = is_tapping

        self.machine = KickStateMachine("robot")

    def reset(self, robot_id: int) -> None:
        """ロボットIDを設定し, Tacticの状態をRUNNINGにリセットする関数."""
        self.robot_id = robot_id
        self.state = TacticState.RUNNING
        self.machine.reset()

    def run(self, world_model: WorldModel) -> MotionCommand:
        """ボールを蹴るためのMotionCommandを生成する関数."""
        command = MotionCommand()
        command.robot_id = self.robot_id
        command.mode = MotionCommand.MODE_NAVI

        # ボールの位置を取得
        ball_pos = world_model.ball.pos
        robot_pos = world_model.robots.our_robots.get(self.robot_id).pos

        # ロボットとボールの距離を計算
        dist_to_ball = tool.get_distance(ball_pos, robot_pos)

        # キック角度を計算
        kick_angle = tool.get_angle(ball_pos, self.target_pos)
        kick_diff_angle = abs(tool.angle_normalize(robot_pos.theta - kick_angle))

        self.machine.update(dist_to_ball, np.rad2deg(kick_diff_angle))

        if self.machine.state == "chasing":
            # ボール側面のどちらかに向かう
            print("chasing")
            if robot_pos.y > ball_pos.y:
                command.desired_pose.y = ball_pos.y + self.CHASING_BALL_APPROACH
            else:
                command.desired_pose.y = ball_pos.y - self.CHASING_BALL_APPROACH
            command.desired_pose.x = ball_pos.x - self.CHASING_BALL_APPROACH * np.cos(kick_angle)
            command.desired_pose.theta = tool.get_angle(robot_pos, ball_pos)

        elif self.machine.state == "aiming":
            # 蹴る方向に向けて移動
            print("aiming")
            command.desired_pose = self.kicking_pose(ball_pos, kick_angle, 0.2)

        elif self.machine.state == "kicking":
            print("kicking")
            command.desired_pose = self.kicking_pose(ball_pos, kick_angle)
            command.kick_power = self.MAX_KICK_POWER
            if self.is_pass:
                command.kick_power = self.pass_power(ball_pos)
            elif self.is_tapping:
                command.kick_power = self.TAPPING_KICK_POWER

        return command

    def kicking_pose(self, ball_pos: State2D, kick_angle: float, dist_ball: float = 0.1) -> State2D:
        """ボールを蹴るための目標位置をを生成"""
        pose = State2D()
        pose.x = ball_pos.x - dist_ball * np.cos(kick_angle)
        pose.y = ball_pos.y - dist_ball * np.sin(kick_angle)
        pose.theta = kick_angle
        return pose

    def pass_power(self, ball_pos: State2D) -> float:
        """ボールからkick_targetまでの距離をもとにキック速度を計算する"""

        MIN_PASS_POWER = 2.0  # ある程度の距離までボールが届き、ロボットがキャッチできる最低速度
        MIN_PASS_DISTANCE = 0.5  # MIN_PASS_POWERで届く最大距離
        MAX_PASS_DISTANCE = 5.0  # MAX_KICK_POWERで届く最大距離

        distance_to_target = tool.get_distance(ball_pos, self.target_pos)

        if distance_to_target < MIN_PASS_DISTANCE:
            return MIN_PASS_POWER
        elif distance_to_target > MAX_PASS_DISTANCE:
            return self.MAX_KICK_POWER
        else:
            # 線形補間
            return MIN_PASS_POWER + (self.MAX_KICK_POWER - MIN_PASS_POWER) * (
                distance_to_target - MIN_PASS_DISTANCE
            ) / (MAX_PASS_DISTANCE - MIN_PASS_DISTANCE)
