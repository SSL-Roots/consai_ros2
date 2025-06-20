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
import copy
from consai_msgs.msg import MotionCommand, State2D

from consai_tools.geometry import geometry_tools as tool

from consai_game.world_model.world_model import WorldModel
from consai_game.core.tactic.tactic_base import TacticBase
from consai_game.utils.generate_dummy_ball_position import generate_dummy_ball_position

from transitions.extensions import GraphMachine


class KickStateMachine(GraphMachine):
    """状態遷移マシン."""

    def __init__(self, name):
        """状態遷移マシンのインスタンスを初期化する関数."""
        self.name = name

        # 状態定義
        states = ["chasing", "aiming", "kicking"]

        # 遷移定義
        transitions = [
            {"trigger": "can_aim", "source": "chasing", "dest": "aiming"},
            {"trigger": "need_rechasing", "source": "aiming", "dest": "chasing"},
            {"trigger": "can_kick", "source": "aiming", "dest": "kicking"},
            {"trigger": "need_reaiming", "source": "kicking", "dest": "aiming"},
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

    def update(self, robot_is_backside: bool, robot_is_on_kick_line: bool) -> None:
        """状態遷移を更新する関数."""
        if self.state == "chasing" and robot_is_backside:
            # ボールの後側に来た
            self.can_aim()

        elif self.state == "aiming" and not robot_is_backside:
            # ボールの前に出てしまった
            self.need_rechasing()

        elif self.state == "aiming" and robot_is_on_kick_line:
            # ロボットが狙いを定める直線上にいるか
            self.can_kick()

        elif self.state == "kicking" and not robot_is_on_kick_line:
            self.need_reaiming()


class Kick(TacticBase):
    """指定した位置にボールを蹴るTactic."""

    # 定数として残すもの（設定ファイルで制御されないもの）
    ANGLE_BALL_TO_ROBOT_THRESHOLD = 120  # ボールが後方に居るとみなす角度[degree]
    ANGLE_FOR_PIVOT_POS = ANGLE_BALL_TO_ROBOT_THRESHOLD + 10  # ボールの後側に移動するための角度[degree]
    DRIBBLE_ON = 1.0  # ドリブルON時の出力
    AIM_VELOCITY_FOR_SETPLAY = 0.2  # セットプレイ時の移動速度[m/s]

    def __init__(self, x=0.0, y=0.0, is_pass=False, is_tapping=False, is_setplay=False):
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
        # 目標値の切り替わりを防ぐための変数
        self.final_target_pos = State2D(x=x, y=y)
        self.is_pass = is_pass
        self.is_tapping = is_tapping

        self.is_setplay = is_setplay

        self.machine = KickStateMachine("robot")

    def reset(self, robot_id: int) -> None:
        """ロボットIDを設定し, Tacticの状態をRUNNINGにリセットする関数."""
        super().reset(robot_id)
        self.machine.reset()

    def run(self, world_model: WorldModel) -> MotionCommand:
        """ボールを蹴るためのMotionCommandを生成する関数."""
        command = MotionCommand()
        command.robot_id = self.robot_id
        command.mode = MotionCommand.MODE_NAVI

        # ロボットの位置を取得
        robot_pos = world_model.robots.our_robots.get(self.robot_id).pos
        # ボールが消えることを想定して、仮想的なボール位置を生成する
        ball_pos = generate_dummy_ball_position(ball=world_model.ball, robot_pos=robot_pos)
        # ロボットとボール間の距離を取得
        dist_robot_to_ball = tool.get_distance(robot_pos, ball_pos)

        width_threshold = 0.1  # ボールからターゲットまでの直線上にロボットが居るかを判定するための幅の閾値
        if self.is_setplay:
            # セットプレイのときは厳しくする
            width_threshold = 0.03

        self.machine.update(
            robot_is_backside=self.robot_is_backside(robot_pos, ball_pos, self.final_target_pos),
            robot_is_on_kick_line=self.robot_is_on_kick_line(
                robot_pos, ball_pos, self.final_target_pos, width_threshold=width_threshold
            ),
        )

        if self.machine.state == "chasing":
            # aiming以降で動作がぶれないように、target_posを保存しておく
            self.final_target_pos = copy.deepcopy(self.target_pos)

            if self.is_setplay or 0.3 < dist_robot_to_ball:
                command.desired_pose = self.move_to_backside_pose(
                    ball_pos=ball_pos,
                    robot_pos=robot_pos,
                    target_pos=self.target_pos,
                    distance=0.3,
                )
                command.desired_pose.theta = tool.get_angle(ball_pos, self.target_pos)
            else:
                command.desired_pose = self.move_to_backside_pose(
                    ball_pos=ball_pos,
                    robot_pos=robot_pos,
                    target_pos=self.target_pos,
                    distance=0.15,
                )
                command.desired_pose.theta = tool.get_angle(robot_pos, ball_pos)
            command.navi_options.avoid_pushing = False

        elif self.machine.state == "aiming":
            # 蹴る方向に向けて移動
            command.navi_options.avoid_pushing = False
            # ドリブラーON
            command.dribble_power = self.DRIBBLE_ON
            if self.is_setplay:
                # セットプレイならちょっと位置を離す
                command.desired_pose = self.kicking_pose(
                    ball_pos=ball_pos, distance=0.25, target_pos=self.final_target_pos
                )
                command.desired_velocity.x = self.AIM_VELOCITY_FOR_SETPLAY  # ゆっくりボールに近づく
            else:
                command.desired_pose = self.kicking_pose(
                    ball_pos=ball_pos, distance=0.15, target_pos=self.final_target_pos
                )

        elif self.machine.state == "kicking":
            # ボールを蹴る
            command.navi_options.avoid_pushing = False
            command.desired_pose = self.kicking_pose(ball_pos=ball_pos, distance=0.04, target_pos=self.final_target_pos)
            # ドリブラーON
            command.dribble_power = self.DRIBBLE_ON
            # キックパワーの設定
            command.kick_power = world_model.game_config.max_kick_power
            if self.is_pass:
                command.kick_power = self.pass_power(
                    ball_pos, target_pos=self.final_target_pos, world_model=world_model
                )
            elif self.is_tapping:
                command.kick_power = world_model.game_config.tapping_kick_power

            if self.is_setplay:
                command.desired_velocity.x = self.AIM_VELOCITY_FOR_SETPLAY  # ゆっくりボールに近づく

        return command

    def robot_is_backside(self, robot_pos: State2D, ball_pos: State2D, target_pos: State2D) -> bool:
        """ボールからターゲットを見て、ロボットが後側に居るかを判定する."""
        # ボールからターゲットへの座標系を作成
        trans = tool.Trans(ball_pos, tool.get_angle(ball_pos, target_pos))
        tr_robot_pos = trans.transform(robot_pos)

        # ボールから見たロボットの位置の角度
        # ボールの後方にいれば角度は90度以上
        tr_ball_to_robot_angle = tool.get_angle(State2D(x=0.0, y=0.0), tr_robot_pos)

        if abs(tr_ball_to_robot_angle) > np.deg2rad(self.ANGLE_BALL_TO_ROBOT_THRESHOLD):
            return True
        return False

    def robot_is_on_kick_line(
        self, robot_pos: State2D, ball_pos: State2D, target_pos: State2D, width_threshold: float
    ) -> bool:
        """ボールからターゲットまでの直線上にロボットが居るかを判定する.

        ターゲットまでの距離が遠いと、角度だけで狙いを定めるのは難しいため、位置を使って判定する.
        """
        MINIMAL_THETA_THRESHOLD = 45  # 最低限満たすべきロボットの角度

        # ボールからターゲットへの座標系を作成
        trans = tool.Trans(ball_pos, tool.get_angle(ball_pos, target_pos))
        tr_robot_pos = trans.transform(robot_pos)
        tr_robot_theta = trans.transform_angle(robot_pos.theta)

        # ボールより前にロボットが居る場合
        if tr_robot_pos.x > 0.0:
            return False

        # ターゲットを向いていない
        if abs(tr_robot_theta) > np.deg2rad(MINIMAL_THETA_THRESHOLD):
            return False

        if abs(tr_robot_pos.y) > width_threshold:
            return False

        return True

    def move_to_backside_pose(
        self, ball_pos: State2D, robot_pos: State2D, target_pos: State2D, distance: float
    ) -> State2D:
        """ボールの後側に移動するための目標位置を生成"""
        # 座標変換クラスのインスタンスの生成
        # ボール中心にボールから目標位置までの角度で変換
        trans = tool.Trans(ball_pos, tool.get_angle(ball_pos, target_pos))
        # ロボットの位置を変換
        tr_robot_pos = trans.transform(robot_pos)

        pivot_angle = np.sign(tr_robot_pos.y) * np.deg2rad(self.ANGLE_FOR_PIVOT_POS)

        tr_pivot_pos = State2D()
        tr_pivot_pos.x = distance * np.cos(pivot_angle)
        tr_pivot_pos.y = distance * np.sin(pivot_angle)

        pose = trans.inverted_transform(tr_pivot_pos)
        pose.theta = tool.get_angle(ball_pos, self.target_pos)
        return pose

    def kicking_pose(self, ball_pos: State2D, target_pos: State2D, distance: float = 0.1) -> State2D:
        """ボールを蹴るための目標位置を生成"""

        # ボールの中心からターゲットへの座標系を作成
        trans = tool.Trans(ball_pos, tool.get_angle(ball_pos, target_pos))

        pose = trans.inverted_transform(State2D(x=-distance, y=0.0))
        pose.theta = tool.get_angle(ball_pos, target_pos)
        return pose

    def pass_power(self, ball_pos: State2D, target_pos: State2D, world_model: WorldModel) -> float:
        """ボールからkick_targetまでの距離をもとにキック速度を計算する"""

        MIN_PASS_DISTANCE = 0.5  # MIN_PASS_POWERで届く最大距離
        MAX_PASS_DISTANCE = 5.0  # MAX_KICK_POWERで届く最大距離

        # 設定ファイルから取得
        MIN_PASS_POWER = world_model.game_config.min_pass_power
        MAX_KICK_POWER = world_model.game_config.max_kick_power

        distance_to_target = tool.get_distance(ball_pos, target_pos)

        if distance_to_target < MIN_PASS_DISTANCE:
            return MIN_PASS_POWER
        elif distance_to_target > MAX_PASS_DISTANCE:
            return MAX_KICK_POWER
        else:
            # 線形補間
            return MIN_PASS_POWER + (MAX_KICK_POWER - MIN_PASS_POWER) * (
                distance_to_target - MIN_PASS_DISTANCE
            ) / (MAX_PASS_DISTANCE - MIN_PASS_DISTANCE)
