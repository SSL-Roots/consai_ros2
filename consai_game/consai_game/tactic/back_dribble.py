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

指定位置へボールをドリブルで運ぶ.
"""

from consai_game.core.tactic.tactic_base import TacticBase
from consai_game.core.tactic.tactic_base import TacticState
from consai_game.world_model.world_model import WorldModel

from consai_msgs.msg import MotionCommand
from consai_msgs.msg import State2D

from consai_tools.geometry import geometry_tools as tool

import numpy as np

from transitions.extensions import GraphMachine


class BackDribbleStateMachine(GraphMachine):
    """ドリブルの状態遷移マシン."""

    # ボールが近いとみなす距離の閾値[m]
    BALL_NEAR_THRESHOLD = 0.5
    # ボールを保持していると判定する距離の閾値[m]
    BALL_GET_THRESHOLD = 0.2
    # 目的地が近いとみなす距離の閾値[m]
    DIST_TARGET_TO_BALL_THRESHOLD = 0.1

    # ドリブル角度の閾値[degree]
    DRIBBLE_ANGLE_THRESHOLD = 5

    def __init__(self, name):
        """状態遷移の初期化."""
        self.name = name

        # 状態定義
        states = ["arrived", "chasing", "aiming", "dribbling"]

        # 遷移定義
        transitions = [
            {"trigger": "start", "source": "arrived", "dest": "chasing"},
            {"trigger": "ball_near", "source": "chasing", "dest": "aiming"},
            {"trigger": "ball_far", "source": "aiming", "dest": "chasing"},
            {"trigger": "dribble", "source": "aiming", "dest": "dribbling"},
            {"trigger": "reaiming", "source": "dribbling", "dest": "aiming"},
            {"trigger": "arrival", "source": "dribbling", "dest": "arrived"},
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
        dist_to_ball: float,
        dist_ball_to_target: float,
        dribble_diff_angle: float,
        ball_trans_pos: float,
        robot_trans_pos: float,
    ):
        """状態遷移."""
        if self.state != "dribbling" and dist_ball_to_target < self.DIST_TARGET_TO_BALL_THRESHOLD:
            self.reset()

        if self.state == "arrived" and self.DIST_TARGET_TO_BALL_THRESHOLD < dist_ball_to_target:
            self.start()

        elif (
            self.state == "chasing"
            and dist_to_ball <= self.BALL_NEAR_THRESHOLD
            and ball_trans_pos.x > robot_trans_pos.x
        ):
            # 状態が"chasing"でボールと目標位置間の距離がしきい値以下かつ
            # ボールの変換後のx座標がロボットの変換後のx座標より大きい場合
            # 状態を"chasing"から"aiming"に遷移
            self.ball_near()

        elif self.state == "aiming" and self.BALL_NEAR_THRESHOLD < dist_to_ball:
            self.ball_far()

        elif (
            self.state == "aiming"
            and dribble_diff_angle < self.DRIBBLE_ANGLE_THRESHOLD
            and dist_to_ball < self.BALL_GET_THRESHOLD
        ):
            self.dribble()

        elif (
            self.state == "dribbling"
            and self.DRIBBLE_ANGLE_THRESHOLD < dribble_diff_angle
            and self.BALL_GET_THRESHOLD <= dist_to_ball
        ):
            self.reaiming()

        elif self.state == "dribbling" and self.BALL_GET_THRESHOLD < dist_to_ball:
            self.reaiming()

        elif self.state == "dribbling" and dist_ball_to_target <= self.DIST_TARGET_TO_BALL_THRESHOLD:
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
    TARGET_MARGIN_DIST = 0.15
    # ボールを運ぶ目標位置のより厳しいマージン[m]
    TARGET_MARGIN_DIST_STRICT = 0.08

    def __init__(self, x=0.0, y=0.0):
        """Initialize the DefendGoal tactic."""
        super().__init__()

        self.move_pos = State2D()
        self.target_pos = State2D(x=x, y=y)
        self.machine = BackDribbleStateMachine("robot")

    def reset(self, robot_id: int) -> None:
        """Reset the tactic state for the specified robot."""
        self.robot_id = robot_id
        self.state = TacticState.RUNNING
        self.machine.reset()

    def run(self, world_model: WorldModel) -> MotionCommand:
        """Run the tactic and return a MotionCommand based on the ball's position and movement."""
        command = MotionCommand()
        command.robot_id = self.robot_id
        command.mode = MotionCommand.MODE_NAVI

        # ボールの位置を取得
        ball_pos = world_model.ball.pos
        # ロボットの位置を取得
        robot_pos = world_model.robots.our_robots.get(self.robot_id).pos

        # ロボットとボールの距離を計算
        dist_to_ball = tool.get_distance(ball_pos, robot_pos)
        # ボールと目標位置の距離を計算
        dist_ball_to_target = tool.get_distance(ball_pos, self.target_pos)

        # ドリブル角度を計算
        dribble_angle = tool.get_angle(self.target_pos, ball_pos)

        # ドリブル角度との差分を計算
        dribble_diff_angle = abs(tool.angle_normalize(robot_pos.theta - dribble_angle))

        # ボールを中心にしターゲットを-x軸とする座標系を生成
        trans = tool.Trans(ball_pos, tool.get_angle(self.target_pos, ball_pos))
        # ボールの座標を変換
        ball_trans_pos = trans.transform(ball_pos)
        # ロボットの座標を変換
        robot_trans_pos = trans.transform(robot_pos)

        # 状態遷移を更新
        self.machine.update(
            dist_to_ball, dist_ball_to_target, np.rad2deg(dribble_diff_angle), ball_trans_pos, robot_trans_pos
        )

        # 基本はボール回避をしない
        command.navi_options.avoid_ball = False

        if self.machine.state == "chasing":
            # ボールの近くへ移動
            self.move_pos.x = ball_pos.x - self.CHASING_BALL_APPROACH_DIST * np.cos(dribble_angle)
            self.move_pos.y = ball_pos.y - self.CHASING_BALL_APPROACH_DIST * np.sin(dribble_angle)
            self.move_pos.theta = tool.get_angle(robot_pos, ball_pos)

            # ドリブラーOFF
            command.dribble_power = self.DRIBBLE_OFF

            # 追いかけるときはボール回避を実行
            command.navi_options.avoid_ball = True

        elif self.machine.state == "aiming":
            # 運搬方向に方向に向けて移動
            self.move_pos.x = ball_pos.x - self.TARGET_MARGIN_DIST_STRICT * np.cos(dribble_angle)
            self.move_pos.y = ball_pos.y - self.TARGET_MARGIN_DIST_STRICT * np.sin(dribble_angle)
            self.move_pos.theta = dribble_angle

            # ドリブラーOFF
            command.dribble_power = self.DRIBBLE_OFF

        elif self.machine.state == "dribbling":
            # 角度が適切な場合はドリブルを実行
            self.move_pos.x = self.target_pos.x - self.TARGET_MARGIN_DIST * np.cos(dribble_angle)
            self.move_pos.y = self.target_pos.y - self.TARGET_MARGIN_DIST * np.sin(dribble_angle)
            self.move_pos.theta = dribble_angle

            # ドリブルON
            command.dribble_power = self.DRIBBLE_ON

        else:
            # 停止処理
            self.move_pos.x = self.target_pos.x - self.TARGET_MARGIN_DIST * np.cos(robot_pos.theta)
            self.move_pos.y = self.target_pos.y - self.TARGET_MARGIN_DIST * np.sin(robot_pos.theta)
            self.move_pos.theta = robot_pos.theta

            # ドリブルOFF
            command.dribble_power = self.DRIBBLE_OFF

        command.desired_pose = self.move_pos

        return command
