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
    """BallApproachの状態遷移マシン."""

    def __init__(self, name):
        """状態遷移マシンのインスタンスを初期化する関数."""
        self.name = name

        # 状態定義
        states = ["chasing", "aiming", "receiving"]

        # 遷移定義
        transitions = [
            {"trigger": "can_aim", "source": "chasing", "dest": "aiming"},
            {"trigger": "need_rechasing", "source": "aiming", "dest": "chasing"},
            {"trigger": "can_receive", "source": "aiming", "dest": "receiving"},
            {"trigger": "need_reaiming", "source": "receiving", "dest": "aiming"},
            {"trigger": "reset", "source": "*", "dest": "chasing"},
        ]

        # ステートマシン構築
        super().__init__(
            model=self,
            states=states,
            transitions=transitions,
            initial="chasing",
            auto_transitions=True,
            ordered_transitions=False,
            title="",
            show_auto_transitions=False,
        )

    def update(self, robot_is_backside: bool, robot_is_on_receive_line: bool) -> None:
        """状態遷移を更新する関数."""
        if self.state == "chasing" and robot_is_backside:
            # ボールの後側に来た
            self.can_aim()

        elif self.state == "aiming" and not robot_is_backside:
            # ボールの前に出てしまった
            self.need_rechasing()

        elif self.state == "aiming" and robot_is_on_receive_line:
            # ロボットが狙いを定める直線上にいるか
            self.can_receive()

        elif self.state == "receiving" and not robot_is_on_receive_line:
            self.need_reaiming()


class BallApproach(TacticBase):
    """ボールアプローチのTactic"""

    # ドリブルON時の出力
    DRIBBLE_ON = 1.0
    # ドリブルOFF時の出力(0.0)
    DRIBBLE_OFF = 0.0

    ANGLE_BALL_TO_ROBOT_THRESHOLD = 120  # ボールが後方に居るとみなす角度[degree]
    ANGLE_FOR_PIVOT_POS = ANGLE_BALL_TO_ROBOT_THRESHOLD + 10  # ボールの後側に移動するための角度[degree]

    def __init__(self):
        """Initialize the DefendGoal tactic"""
        super().__init__()
        self.move_pos = State2D()
        self.machine = BallApproachStateMachine("robot")

    def reset(self, robot_id: int) -> None:
        """Reset the tactic state for the specified robot"""
        self.robot_id = robot_id
        self.machine.reset()

    def append_machine_state_to_name(self) -> None:
        """状態遷移マシンの状態を名前に追加する関数."""
        self.name = f"{self.__class__.__name__}.{self.machine.state}"

    def run(self, world_model: WorldModel) -> MotionCommand:
        """Run the tactic and return a MotionCommand based on the ball's position and movement"""
        command = MotionCommand()
        command.robot_id = self.robot_id
        command.mode = MotionCommand.MODE_NAVI

        # 基本はボール回避をしない
        command.navi_options.avoid_ball = False
        # 基本はドリブルOFF
        command.dribble_power = self.DRIBBLE_OFF

        # ボールの位置を取得
        ball_pos = world_model.ball.pos
        # ボールの予想到達地点を取得
        ball_stop_pos = world_model.ball_activity.ball_stop_position

        # ロボットの位置を取得
        robot_pos = world_model.robots.our_robots.get(self.robot_id).pos

        self.machine.update(
            robot_is_backside=self.robot_is_backside(robot_pos, ball_pos, ball_stop_pos),
            robot_is_on_receive_line=self.robot_is_on_receiving_line(robot_pos, ball_pos, ball_stop_pos),
        )

        if self.machine.state == "chasing":
            command.desired_pose = self.move_to_backside_pose(
                robot_pos=robot_pos,
                ball_pos=ball_pos,
                ball_stop_pos=ball_stop_pos,
                distance=0.3,
            )
            command.desired_pose.theta = tool.get_angle(ball_stop_pos, ball_pos)
            command.navi_options.avoid_pushing = False

        elif self.machine.state == "aiming":
            # 受け取る方向に向けて移動
            command.navi_options.avoid_pushing = False
            command.desired_pose = self.receiving_pose(ball_pos, ball_stop_pos, distance=0.15)
            command.dribble_power = self.DRIBBLE_ON

        elif self.machine.state == "receiving":
            # ボールを受け取る
            command.navi_options.avoid_pushing = False
            command.desired_pose = self.receiving_pose(ball_pos, ball_stop_pos, distance=0.04)
            command.dribble_power = self.DRIBBLE_ON

        self.append_machine_state_to_name()  # デバッグのため、状態を名前に追加
        return command

    def robot_is_backside(self, robot_pos: State2D, ball_pos: State2D, ball_stop_pos: State2D) -> bool:
        """ボールからターゲットを見て、ロボットが後側に居るかを判定する."""
        # ボール最終目標地点からボールへの座標系を作成
        trans = tool.Trans(ball_stop_pos, tool.get_angle(ball_stop_pos, ball_pos))
        tr_robot_pos = trans.transform(robot_pos)

        # ボールから見たロボットの位置の角度
        # ボールの後方にいれば角度は90度以上
        tr_ball_to_robot_angle = tool.get_angle(State2D(x=0.0, y=0.0), tr_robot_pos)

        if abs(tr_ball_to_robot_angle) > np.deg2rad(self.ANGLE_BALL_TO_ROBOT_THRESHOLD):
            return True
        return False

    def robot_is_on_receiving_line(self, robot_pos: State2D, ball_pos: State2D, ball_stop_pos: State2D) -> bool:
        """ボールからターゲットまでの直線上にロボットが居るかを判定する.

        ターゲットまでの距離が遠いと、角度だけで狙いを定めるのは難しいため、位置を使って判定する.
        """
        MINIMAL_THETA_THRESHOLD = 45  # 最低限満たすべきロボットの角度
        WIDTH_THRESHOLD = 0.03  # 直線に乗っているかの距離

        # ボールからターゲットへの座標系を作成
        trans = tool.Trans(ball_pos, tool.get_angle(ball_stop_pos, ball_pos))
        tr_robot_pos = trans.transform(robot_pos)

        # ボールより前にロボットが居る場合
        if tr_robot_pos.x > 0.0:
            return False

        # ターゲットを向いていない
        if abs(tr_robot_pos.theta) > np.deg2rad(MINIMAL_THETA_THRESHOLD):
            return False

        if abs(tr_robot_pos.y) > WIDTH_THRESHOLD:
            return False

        return True

    def move_to_backside_pose(
        self, robot_pos: State2D, ball_pos: State2D, ball_stop_pos: State2D, distance: float
    ) -> State2D:
        """ボールの後側に移動するための目標位置を生成"""
        # 座標変換クラスのインスタンスの生成
        # ボール中心にボールから目標位置までの角度で変換
        trans = tool.Trans(ball_stop_pos, tool.get_angle(ball_stop_pos, ball_pos))
        # ロボットの位置を変換
        tr_robot_pos = trans.transform(robot_pos)

        pivot_angle = np.sign(tr_robot_pos.y) * np.deg2rad(self.ANGLE_FOR_PIVOT_POS)

        tr_pivot_pos = State2D()
        tr_pivot_pos.x = distance * np.cos(pivot_angle)
        tr_pivot_pos.y = distance * np.sin(pivot_angle)

        pose = trans.inverted_transform(tr_pivot_pos)
        pose.theta = tool.get_angle(ball_stop_pos, ball_pos)
        return pose

    def receiving_pose(self, ball_pos: State2D, ball_stop_pos: State2D, distance: float = 0.1) -> State2D:
        """ボールを受け取るための目標位置を生成"""

        # ボールの中心からターゲットへの座標系を作成
        trans = tool.Trans(ball_stop_pos, tool.get_angle(ball_stop_pos, ball_pos))

        pose = trans.inverted_transform(State2D(x=-distance, y=0.0))
        pose.theta = tool.get_angle(ball_stop_pos, ball_pos)
        return pose
