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
from consai_game.tactic.ball_approach import BallApproach

from consai_msgs.msg import MotionCommand
from consai_msgs.msg import State2D

from consai_tools.geometry import geometry_tools as tool

import numpy as np

from transitions.extensions import GraphMachine


class DribbleStateMachine(GraphMachine):
    """ドリブルの状態遷移マシン."""

    # ボールが近いとみなす距離の閾値[m]
    BALL_NEAR_THRESHOLD = 0.5
    # ボールを保持していると判定する距離の閾値[m]
    BALL_GET_THRESHOLD = 0.2
    # 目的地が近いとみなす距離の閾値[m]
    DIST_TARGET_TO_BALL_THRESHOLD = 0.1

    DRIBBLE_DIST = 0.2

    # ドリブル角度の閾値[degree]
    DRIBBLE_ANGLE_THRESHOLD = 5

    def __init__(self, name):
        """状態遷移の初期化."""
        self.name = name

        # 状態定義
        states = ["arrived", "approaching", "dribbling"]

        # 遷移定義
        transitions = [
            {"trigger": "approach", "source": "arrived", "dest": "approaching"},
            {"trigger": "dribble", "source": "approaching", "dest": "dribbling"},
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
        dribble_diff_angle: float,
        ball_is_front: bool,
        approach_finish: bool,
    ):
        """状態遷移."""
        if self.state == "arrived" and self.DIST_TARGET_TO_BALL_THRESHOLD < dist_ball_to_target:
            self.approach()

        elif self.state == "approaching" and approach_finish:
            self.dribble()

        elif self.state == "dribbling" and (
            self.DRIBBLE_ANGLE_THRESHOLD < dribble_diff_angle
            or self.DRIBBLE_DIST < dist_robot_to_ball
            or ball_is_front is False
        ):
            self.reapproach()

        elif self.state == "dribbling" and dist_ball_to_target < self.DIST_TARGET_TO_BALL_THRESHOLD:
            self.arrival()


class Dribble(TacticBase):
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

        self.move_pos = State2D()
        self.target_pos = State2D(x=x, y=y)
        self.machine = DribbleStateMachine("robot")

        self.ball_approach = BallApproach(x=x, y=y)

    def reset(self, robot_id: int) -> None:
        """Reset the tactic state for the specified robot."""
        self.robot_id = robot_id
        self.state = TacticState.RUNNING
        self.machine.reset()
        self.ball_approach.reset(robot_id)

    def run(self, world_model: WorldModel) -> MotionCommand:
        """Run the tactic and return a MotionCommand based on the ball's position and movement."""
        # ボールの位置を取得
        ball_pos = world_model.ball.pos
        # ロボットの位置を取得
        robot_pos = world_model.robots.our_robots.get(self.robot_id).pos

        # ロボットとボールの距離を計算
        dist_robot_to_ball = tool.get_distance(ball_pos, robot_pos)
        # ボールと目標位置の距離を計算
        dist_ball_to_target = tool.get_distance(ball_pos, self.target_pos)

        # ドリブル角度を計算
        dribble_angle = tool.get_angle(ball_pos, self.target_pos)

        # ドリブル角度との差分を計算
        dribble_diff_angle = abs(tool.angle_normalize(robot_pos.theta - dribble_angle))

        # ロボットを中心に、ターゲットを+x軸とした座標系を作る
        trans = tool.Trans(robot_pos, tool.get_angle(robot_pos, self.target_pos))
        tr_ball_pos = trans.transform(ball_pos)
        # ボールがロボットの正面にあるか
        ball_is_front = tr_ball_pos.x > 0.0

        # ball_approachが完了したか
        approach_finish = self.ball_approach.machine.state == "arrived"

        # 状態遷移を更新
        self.machine.update(
            dist_robot_to_ball=dist_robot_to_ball,
            dist_ball_to_target=dist_ball_to_target,
            dribble_diff_angle=np.rad2deg(dribble_diff_angle),
            ball_is_front=ball_is_front,
            approach_finish=approach_finish,
        )

        command = MotionCommand()
        command.robot_id = self.robot_id
        command.mode = MotionCommand.MODE_NAVI

        # 基本はボール回避をしない
        command.navi_options.avoid_ball = False

        if self.machine.state == "approaching":
            command = self.ball_approach.run(world_model)

            # ドリブルOFF
            command.dribble_power = self.DRIBBLE_OFF
        elif self.machine.state == "dribbling":
            command = self.dribble_the_ball(command=command, world_model=world_model)
        else:
            command = self.dribble_the_ball(command=command, world_model=world_model)
            # ドリブルOFF
            command.dribble_power = self.DRIBBLE_OFF

        return command

    def dribble_the_ball(self, command: MotionCommand, world_model: WorldModel) -> MotionCommand:
        """ボールをドリブルするコマンドを返す."""
        DRIBBLING_VELOCITY = 0.5  # ドリブル時の速度[m/s]
        PLACING_VELOCITY = 0.1  # ボールを置くときの速度[m/s]
        PLACING_THRESHOLD = 0.5  # placingを始める距離

        ball_pos = world_model.ball.pos
        robot_pos = world_model.robots.our_robots.get(self.robot_id).pos
        dist_ball_to_target = tool.get_distance(ball_pos, self.target_pos)

        # ボールと目標位置が離れている場合は、ボールを基準に目標位置を生成する
        if dist_ball_to_target > PLACING_THRESHOLD:
            command.desired_pose = self.dribbling_pose_from_ball(
                target_pos=self.target_pos, ball_pos=ball_pos, distance=self.TARGET_MARGIN_DIST
            )

            # 走行速度を落としてドリブルする
            command.desired_velocity.x = DRIBBLING_VELOCITY
            command.dribble_power = self.DRIBBLE_ON
        else:
            # ボールと目標位置が近い場合は、ロボットを基準に目標位置を生成する
            # これをしなければ、ボールと目標位置が超接近し、目標位置が不安定になってしまう
            command.desired_pose = self.dribbling_pose_from_robot(
                target_pos=self.target_pos, robot_pos=robot_pos, distance=self.TARGET_MARGIN_DIST
            )

            # 走行速度を落として、ボールを置く
            command.desired_velocity.x = PLACING_VELOCITY
            # place後にボールをこぼさないように、ドリブルを止める
            command.dribble_power = self.DRIBBLE_OFF

        return command

    def dribbling_pose_from_ball(self, target_pos: State2D, ball_pos: State2D, distance: float) -> State2D:
        """ボールをドリブルするための目標位置を生成.

        ボールとターゲットを結ぶ直線を作るため、ターゲットとの距離が近づくと位置生成が不安定になる.
        """

        # ボールを中心にターゲットを+X軸とした座標系を作る
        trans = tool.Trans(ball_pos, tool.get_angle(ball_pos, target_pos))
        # ボールからdistanceだけ前に進んだ位置を目標にする
        pose = trans.inverted_transform(State2D(x=distance, y=0.0))
        pose.theta = trans.inverted_transform_angle(0.0)  # ボールからターゲットを見る角度

        return pose

    def dribbling_pose_from_robot(self, target_pos: State2D, robot_pos: State2D, distance: float) -> State2D:
        """ボールをドリブルするための目標位置を生成.

        ロボットとターゲット位置をもとに目標位置を生成するため、ボールがロボット前方にない場合はドリブルに失敗する.
        """
        # ロボットを中心にターゲットを+X軸とした座標系を作る
        trans = tool.Trans(robot_pos, tool.get_angle(robot_pos, target_pos))
        # ロボットからdistanceだけ前に進んだ位置を目標にする
        pose = trans.inverted_transform(State2D(x=distance, y=0.0))
        pose.theta = trans.inverted_transform_angle(0.0)  # ロボットからターゲットを見る角度

        return pose
