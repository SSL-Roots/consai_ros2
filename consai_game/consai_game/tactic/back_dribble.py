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
Dribble Tactic.

指定位置へボールを後ろ向きのドリブルで運ぶ.
"""

from consai_game.core.tactic.tactic_base import TacticBase
from consai_game.core.tactic.tactic_base import TacticState
from consai_game.world_model.world_model import WorldModel
from consai_game.utils.generate_dummy_ball_position import generate_dummy_ball_position

from consai_msgs.msg import MotionCommand
from consai_msgs.msg import State2D

from consai_tools.geometry import geometry_tools as tool

import numpy as np

from transitions.extensions import GraphMachine
from typing import Optional


class DribbleStateMachine(GraphMachine):
    """ドリブルの状態遷移マシン."""

    # 目的地が近いとみなす距離の閾値[m]
    DIST_TARGET_TO_BALL_THRESHOLD = 0.1

    DRIBBLE_DIST = 0.2

    # ドリブル角度の閾値[degree]
    DRIBBLE_ANGLE_THRESHOLD = 5

    def __init__(self, name):
        """状態遷移の初期化."""
        self.name = name

        # 状態定義
        states = ["arrived", "approaching", "catching", "dribbling"]

        # 遷移定義
        transitions = [
            {"trigger": "approach", "source": "arrived", "dest": "approaching"},
            {"trigger": "catch", "source": "approaching", "dest": "catching"},
            {"trigger": "dribble", "source": "catching", "dest": "dribbling"},
            {"trigger": "need_approach", "source": "catching", "dest": "approaching"},
            {"trigger": "arrival", "source": "dribbling", "dest": "arrived"},
            {"trigger": "reapproach", "source": "dribbling", "dest": "approaching"},
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

    def update(
        self,
        dist_robot_to_ball: float,
        dist_ball_to_target: float,
        ball_is_front: bool,
        robot_has_ball: bool,
        ball_is_far: bool,
    ):
        """状態遷移."""
        if self.state == "arrived" and self.DIST_TARGET_TO_BALL_THRESHOLD < dist_ball_to_target:
            # ボールがロボットから離れすぎている
            self.approach()

        elif self.state == "approaching" and ball_is_front:
            # ボールがロボットの正面にある
            self.catch()

        elif self.state == "catching" and robot_has_ball:
            # ロボットがボールを持っている
            self.dribble()

        elif self.state == "catching" and ball_is_far:
            # ロボットがボールから離れた
            self.need_approach()

        elif self.state == "dribbling" and (self.DRIBBLE_DIST < dist_robot_to_ball):
            # ロボットがドリブル角度から外れる
            # または、ロボットがボールから離れすぎている
            self.reapproach()

        elif self.state == "dribbling" and dist_ball_to_target < self.DIST_TARGET_TO_BALL_THRESHOLD:
            # ボールが目的位置に到着する
            self.arrival()


class BackDribble(TacticBase):
    """指定した位置にドリブルするTactic."""

    # ドリブルON時の出力
    DRIBBLE_ON = 1.0
    # ドリブルOFF時の出力(0.0)
    DRIBBLE_OFF = 0.0

    # ボール追跡時の回り込みの距離[m]
    CHASING_BALL_APPROACH_DIST = 0.5
    # ボールを運ぶ目標位置のマージン[m]
    TARGET_MARGIN_DIST = 0.1

    def __init__(self, x=0.0, y=0.0):
        """Initialize the DefendGoal tactic."""
        super().__init__()

        self.target_pos = State2D(x=x, y=y)
        self.machine = DribbleStateMachine("robot")

    def reset(self, robot_id: int) -> None:
        """Reset the tactic state for the specified robot."""
        self.robot_id = robot_id
        self.state = TacticState.RUNNING
        self.machine.reset()

    def run(self, world_model: WorldModel) -> MotionCommand:
        """Run the tactic and return a MotionCommand based on the ball's position and movement."""
        # ロボットの位置を取得
        robot_pos = world_model.robots.our_robots.get(self.robot_id).pos
        # ボールが消えることを想定して、仮想的なボール位置を生成する
        ball_pos = generate_dummy_ball_position(ball=world_model.ball, robot_pos=robot_pos)
        # ロボットとボールの距離を計算
        dist_robot_to_ball = tool.get_distance(ball_pos, robot_pos)
        # ボールと目標位置の距離を計算
        dist_ball_to_target = tool.get_distance(ball_pos, self.target_pos)

        # 状態遷移を更新
        APPROACH_DISTANCE = 0.3  # ボールに近づく距離
        BALL_IS_FAR_THRESHOLD = 0.5  # ボールがロボットから離れすぎているとみなす距離
        ball_is_far = dist_robot_to_ball > BALL_IS_FAR_THRESHOLD
        self.machine.update(
            dist_robot_to_ball=dist_robot_to_ball,
            dist_ball_to_target=dist_ball_to_target,
            ball_is_front=self.ball_is_front(
                ball_pos=ball_pos, robot_pos=robot_pos, dist_threshold=APPROACH_DISTANCE + 0.1
            ),  # マージンをもたせる
            robot_has_ball=self.ball_is_front(ball_pos=ball_pos, robot_pos=robot_pos, dist_threshold=0.11),
            ball_is_far=ball_is_far,
        )

        command = MotionCommand()
        command.robot_id = self.robot_id
        command.mode = MotionCommand.MODE_NAVI

        # 基本はボール回避をしない
        command.navi_options.avoid_ball = False

        if self.machine.state == "approaching":
            command = self.approach_to_ball(
                command=command, ball_pos=ball_pos, distance=APPROACH_DISTANCE, avoid_ball=True
            )
        elif self.machine.state == "catching":
            # ゆっくり近づく
            command = self.approach_to_ball(
                command=command, ball_pos=ball_pos, distance=0.0, avoid_ball=False, velocity=0.1
            )
        elif self.machine.state == "dribbling":
            command = self.dribble_the_ball(command=command, ball_pos=ball_pos, robot_pos=robot_pos, velocity=0.1)
        else:
            command = self.dribble_the_ball(command=command, ball_pos=ball_pos, robot_pos=robot_pos)
            # ドリブルOFF
            command.dribble_power = self.DRIBBLE_OFF

        return command

    def ball_is_front(self, ball_pos: State2D, robot_pos: State2D, dist_threshold: float) -> bool:
        SIDE_DIST_THRESHOLD = 0.05  # 横方向にどれだけ離れることを許容するか
        THETA_THRESHOLD = 5  # 最低限守るべきロボットの姿勢 deg

        # ボールを中心に、ターゲットを+x軸とした座標系を作る
        trans = tool.Trans(ball_pos, tool.get_angle(ball_pos, self.target_pos))
        tr_robot_pos = trans.transform(robot_pos)
        tr_robot_theta = trans.transform_angle(robot_pos.theta)

        # ロボットがボールの後ろにいる
        if tr_robot_pos.x < 0:
            return False

        # ボールが正面から離れすぎている
        if tr_robot_pos.x > dist_threshold:
            return False

        # ロボットがボールを見ていない
        if abs(tr_robot_theta) < np.deg2rad(180 - THETA_THRESHOLD):
            return False

        # ボールが横方向に離れすぎている
        if abs(tr_robot_pos.y) > SIDE_DIST_THRESHOLD:
            return False
        return True

    def approach_to_ball(
        self,
        command: MotionCommand,
        ball_pos: State2D,
        distance: float,
        avoid_ball: bool,
        velocity: Optional[float] = None,
    ) -> MotionCommand:
        """ボールに近づくコマンドを返す."""

        # ボールを中心にターゲットを+X軸とした座標系を作る
        trans = tool.Trans(ball_pos, tool.get_angle(ball_pos, self.target_pos))

        # ボールの前に目標位置を生成する
        command.desired_pose = trans.inverted_transform(State2D(x=distance, y=0.0))
        command.desired_pose.theta = tool.get_angle(self.target_pos, ball_pos)  # ターゲットからボールを見る

        # ボール回避をONにして、ボールに回り込む
        if avoid_ball:
            command.navi_options.avoid_ball = True

        # 移動速度を制限する
        if velocity:
            command.desired_velocity.x = velocity

        # ドリブルON
        command.dribble_power = self.DRIBBLE_ON
        return command

    def dribble_the_ball(
        self, command: MotionCommand, ball_pos: State2D, robot_pos: State2D, velocity: Optional[float] = None
    ) -> MotionCommand:
        """ボールをドリブルするコマンドを返す."""
        DRIBBLING_POS = 0.5  # 適当にボールから離れた位置を指定する。

        # ロボットを中心にターゲットを+X軸とした座標系を作る
        trans = tool.Trans(robot_pos, tool.get_angle(robot_pos, self.target_pos))

        # ロボットの後ろに目標位置を生成する
        command.desired_pose = trans.inverted_transform(State2D(x=DRIBBLING_POS, y=0.0))
        command.desired_pose.theta = tool.get_angle(self.target_pos, robot_pos)  # ターゲットからロボットを見る

        # 移動速度を制限する
        if velocity:
            command.desired_velocity.x = velocity

        # ドリブルON
        command.dribble_power = self.DRIBBLE_ON
        return command
