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
条件に応じてキックやパスを切り替えるTactic
"""

from consai_game.core.tactic.tactic_base import TacticBase
from consai_game.tactic.dribble import Dribble
from consai_game.tactic.position import Position
from consai_game.world_model.world_model import WorldModel
from consai_tools.geometry import geometry_tools as tools

from consai_msgs.msg import MotionCommand
from consai_msgs.msg import State2D


class CompositeBallPlacement(TacticBase):
    def __init__(self):
        super().__init__()
        self.tactic_dribble = Dribble()
        self.tactic_position = Position()

    def reset(self, robot_id: int) -> None:
        """Reset the tactic state for the specified robot."""
        super().reset(robot_id)

        # 所有するTacticも初期化する
        self.tactic_dribble.reset(robot_id)
        self.tactic_position.reset(robot_id)

    def run(self, world_model: WorldModel) -> MotionCommand:
        """状況に応じて実行するtacticを切り替えてrunする."""

        # ボールがプレースメント位置についたら
        if world_model.ball_activity.ball_is_on_placement_area:
            # その場にとどまる
            return self.stay(world_model)

        # ボールに一番近かったら
        if world_model.robot_activity.our_robots_by_ball_distance[0] == self.robot_id:
            # ボールをドリブルする
            self.tactic_dribble.target_pos = world_model.referee.placement_pos
            return self.tactic_dribble.run(world_model)

        # プレースメント位置に一番近かったら
        if world_model.robot_activity.our_robots_by_placement_distance[0] == self.robot_id:
            # ボールとプレースメント位置を結ぶ直線の後ろに移動する
            return self.support_placement(world_model)

        # それ以外の場合は
        # プレースメントエリア回避ONで、その場にとどまる
        return self.stay(world_model)

    def stay(self, world_model: WorldModel) -> MotionCommand:
        """その場にとどまるコマンドを返す."""
        robot_pos = world_model.robots.our_visible_robots[self.robot_id].pos

        command = MotionCommand()
        command.robot_id = self.robot_id
        command.mode = MotionCommand.MODE_NAVI
        # 目標位置をロボット自身の座標にして、その場にとどまる
        command.desired_pose = robot_pos

        # プレースメントエリア回避ON
        command.navi_options.avoid_placement_area = True

        return command

    def support_placement(self, world_model: WorldModel) -> MotionCommand:
        """プレースメントをサポートするコマンドを返す."""
        ball_pos = world_model.ball.pos
        placement_pos = world_model.referee.placement_pos

        # placement_posとball_posを結ぶ直線をX+軸にした座標系を作る
        trans = tools.Trans(placement_pos, tools.get_angle(placement_pos, ball_pos))

        command = MotionCommand()
        command.robot_id = self.robot_id
        command.mode = MotionCommand.MODE_NAVI
        # ボールを受け取れるように、プレースメント位置の後ろに移動する
        command.desired_pose = trans.inverted_transform(State2D(x=-0.1, y=0.0))
        command.desired_pose.theta = trans.inverted_transform_angle(0.0)

        return command
