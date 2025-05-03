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

    MAX_KICK_POWER = 6.0  # 6.5 m/sを越えてはいけない
    # 1m 以上ボールを持って移動すると、Excessive Dribbling違反になる
    TAPPING_KICK_POWER = 2.0  # ボールをコツコツ蹴ってドリブルするためのキックパワー

    ANGLE_BALL_TO_ROBOT_THRESHOLD = 120  # ボールが後方に居るとみなす角度[degree]
    ANGLE_FOR_PIVOT_POS = ANGLE_BALL_TO_ROBOT_THRESHOLD + 10  # ボールの後側に移動するための角度[degree]

    def __init__(self, x=0.0, y=0.0, is_pass=False, is_tapping=False, is_setplay=True):
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

        self.is_setplay = is_setplay

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
        # ロボットの位置を取得
        robot_pos = world_model.robots.our_robots.get(self.robot_id).pos
        # ロボットとボール間の距離を取得
        dist_robot_to_ball = tool.get_distance(robot_pos, ball_pos)

        self.machine.update(
            robot_is_backside=self.robot_is_backside(robot_pos, ball_pos),
            robot_is_on_kick_line=self.robot_is_on_kick_line(robot_pos, ball_pos),
        )

        if self.machine.state == "chasing":
            if world_model.ball_activity.is_their_team_ball_holder:
                # 相手がボールを持っている場合は目標位置をボールと自チームのゴールの直線上に設定
                command.desired_pose = self.move_to_pose_with_protecting_our_goal(
                    ball_pos=ball_pos,
                    robot_pos=robot_pos,
                    their_holder=world_model.ball_activity.ball_holder.robot.pos,
                    distance=0.15,
                )
            else:
                if self.is_setplay or 0.3 < dist_robot_to_ball:
                    command.desired_pose = self.move_to_backside_pose(
                        ball_pos=ball_pos,
                        robot_pos=robot_pos,
                        distance=0.3,
                    )
                else:
                    command.desired_pose = self.move_to_backside_pose(
                        ball_pos=ball_pos,
                        robot_pos=robot_pos,
                        distance=0.15,
                    )
                command.desired_pose.theta = tool.get_angle(robot_pos, ball_pos)
                command.navi_options.avoid_pushing = False

        elif self.machine.state == "aiming":
            # 蹴る方向に向けて移動
            command.navi_options.avoid_pushing = False
            command.desired_pose = self.kicking_pose(ball_pos=ball_pos, distance=0.15)

        elif self.machine.state == "kicking":
            # ボールを蹴る
            command.navi_options.avoid_pushing = False
            command.desired_pose = self.kicking_pose(ball_pos=ball_pos, distance=0.08)
            command.kick_power = self.MAX_KICK_POWER
            if self.is_pass:
                command.kick_power = self.pass_power(ball_pos)
            elif self.is_tapping:
                command.kick_power = self.TAPPING_KICK_POWER

        return command

    def robot_is_backside(self, robot_pos: State2D, ball_pos: State2D) -> bool:
        """ボールからターゲットを見て、ロボットが後側に居るかを判定する."""
        # ボールからターゲットへの座標系を作成
        trans = tool.Trans(ball_pos, tool.get_angle(ball_pos, self.target_pos))
        tr_robot_pos = trans.transform(robot_pos)

        # ボールから見たロボットの位置の角度
        # ボールの後方にいれば角度は90度以上
        tr_ball_to_robot_angle = tool.get_angle(State2D(x=0.0, y=0.0), tr_robot_pos)

        if abs(tr_ball_to_robot_angle) > np.deg2rad(self.ANGLE_BALL_TO_ROBOT_THRESHOLD):
            return True
        return False

    def robot_is_on_kick_line(self, robot_pos: State2D, ball_pos: State2D) -> bool:
        """ボールからターゲットまでの直線上にロボットが居るかを判定する.

        ターゲットまでの距離が遠いと、角度だけで狙いを定めるのは難しいため、位置を使って判定する.
        """
        MINIMAL_THETA_THRESHOLD = 45  # 最低限満たすべきロボットの角度
        WIDTH_THRESHOLD = 0.03  # 直線に乗っているかの距離

        # ボールからターゲットへの座標系を作成
        trans = tool.Trans(ball_pos, tool.get_angle(ball_pos, self.target_pos))
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

    def move_to_backside_pose(self, ball_pos: State2D, robot_pos: State2D, distance: float) -> State2D:
        """ボールの後側に移動するための目標位置を生成"""
        # 座標変換クラスのインスタンスの生成
        # ボール中心にボールから目標位置までの角度で変換
        trans = tool.Trans(ball_pos, tool.get_angle(ball_pos, self.target_pos))
        # ロボットの位置を変換
        tr_robot_pos = trans.transform(robot_pos)

        pivot_angle = np.sign(tr_robot_pos.y) * np.deg2rad(self.ANGLE_FOR_PIVOT_POS)

        tr_pivot_pos = State2D()
        tr_pivot_pos.x = distance * np.cos(pivot_angle)
        tr_pivot_pos.y = distance * np.sin(pivot_angle)

        pose = trans.inverted_transform(tr_pivot_pos)
        pose.theta = tool.get_angle(ball_pos, self.target_pos)
        return pose

    def move_to_pose_with_protecting_our_goal(
        self, ball_pos: State2D, robot_pos: State2D, their_holder: State2D, distance: float
    ) -> State2D:
        """自チームのゴール中心とボールを結んだ直線上に目標位置を生成する"""
        # 座標変換クラスのインスタンスの生成
        # ボール中心にボールから目標位置までの角度で変換
        trans = tool.Trans(ball_pos, tool.get_angle(ball_pos, their_holder))

        tr_pivot_pos = State2D()
        tr_pivot_pos.x = -distance

        pose = trans.inverted_transform(tr_pivot_pos)
        pose.theta = tool.get_angle(pose, ball_pos)
        return pose

    def kicking_pose(self, ball_pos: State2D, distance: float = 0.1) -> State2D:
        """ボールを蹴るための目標位置を生成"""

        # ボールの中心からターゲットへの座標系を作成
        trans = tool.Trans(ball_pos, tool.get_angle(ball_pos, self.target_pos))

        pose = trans.inverted_transform(State2D(x=-distance, y=0.0))
        pose.theta = tool.get_angle(ball_pos, self.target_pos)
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
