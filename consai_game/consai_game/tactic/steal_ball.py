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

from consai_game.world_model.world_model import WorldModel
from consai_game.core.tactic.tactic_base import TacticBase
from consai_game.utils.generate_dummy_ball_position import generate_dummy_ball_position

from consai_msgs.msg import MotionCommand, State2D
from consai_tools.geometry import geometry_tools as tool


class StealBall(TacticBase):
    """敵がボールを持っている前提で、それを奪うTacctic."""

    # ドリブルON時の出力
    DRIBBLE_ON = 1.0

    def __init__(self):
        """インスタンスを初期化する関数."""
        super().__init__()

    def run(self, world_model: WorldModel) -> MotionCommand:
        """指定した位置に移動するためのMotionCommandを生成する関数."""
        # ロボットの位置を取得
        robot_pos = world_model.robots.our_robots.get(self.robot_id).pos
        # ボールが消えることを想定して、仮想的なボール位置を生成する
        ball_pos = generate_dummy_ball_position(ball=world_model.ball, robot_pos=robot_pos)

        command = MotionCommand()
        command.robot_id = self.robot_id
        command.mode = MotionCommand.MODE_NAVI

        # 相手がボールを持っている場合
        if world_model.ball_activity.is_their_team_ball_holder:
            their_pos = world_model.ball_activity.ball_holder.robot.pos
            # ボールを奪う
            command = self.steal_the_ball(
                command=command, ball_pos=ball_pos, their_pos=their_pos, robot_pos=robot_pos, distance=0.05
            )

        else:
            # 相手がボール持っていない場合は
            # ボールにまっすぐ接近する
            command = self.move_to_the_ball(command=command, ball_pos=ball_pos, robot_pos=robot_pos, distance=0.09)

        return command

    def steal_the_ball(
        self, command: MotionCommand, ball_pos: State2D, their_pos: State2D, robot_pos: State2D, distance: float
    ) -> MotionCommand:
        """ボールを奪うためのMotionCommandを生成する関数."""
        print("steal_the_ball")

        # ボールと相手ロボットを結び座標系をつくり
        # 相手の正面に行く動作を作る
        trans = tool.Trans(ball_pos, tool.get_angle(ball_pos, their_pos))

        command.desired_pose = trans.inverted_transform(State2D(x=-distance, y=0.0))
        command.desired_pose.theta = tool.get_angle(ball_pos, their_pos)  # ボールから相手を見る角度

        # ドリブルを回す
        command.dribble_power = self.DRIBBLE_ON

        # 相手と味方が近い場合はロボットを避けない
        DISTANCE_THRESHOLD = 0.1
        if tool.get_distance(robot_pos, their_pos) < DISTANCE_THRESHOLD:
            command.navi_options.avoid_their_robots = False
            command.navi_options.avoid_pushing = False

        # ボールを避けない
        command.navi_options.avoid_ball = False

        return command

    def move_to_the_ball(
        self, command: MotionCommand, ball_pos: State2D, robot_pos: State2D, distance: float
    ) -> MotionCommand:
        """ボールを奪うためのMotionCommandを生成する関数."""

        # ボールからロボットへの座標系をつくり
        # ボールに近づく

        trans = tool.Trans(ball_pos, tool.get_angle(ball_pos, robot_pos))
        command.desired_pose = trans.inverted_transform(State2D(x=distance, y=0.0))
        command.desired_pose.theta = tool.get_angle(robot_pos, ball_pos)  # ロボットからボールを見る角度

        # ドリブルを回す
        command.dribble_power = self.DRIBBLE_ON

        # ボールを避けない
        command.navi_options.avoid_ball = False

        return command
