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
from consai_game.tactic.stay import Stay
from consai_game.tactic.wrapper.forbid_moving_in_placement_area import ForbidMovingInPlacementArea
from consai_game.world_model.world_model import WorldModel
from consai_tools.geometry import geometry_tools as tools

from consai_msgs.msg import MotionCommand
from consai_msgs.msg import State2D


class CompositeBallPlacement(TacticBase):
    DRIBBLE_VELOCITY = 0.5  # ドリブル時の移動速度 [m/s]

    def __init__(self):
        super().__init__()
        self.tactic_dribble = Dribble()
        self.tactic_avoid_and_stay = ForbidMovingInPlacementArea(tactic=Stay())

    def reset(self, robot_id: int) -> None:
        """Reset the tactic state for the specified robot."""
        super().reset(robot_id)

        # 所有するTacticも初期化する
        self.tactic_dribble.reset(robot_id)
        self.tactic_avoid_and_stay.reset(robot_id)

    def run(self, world_model: WorldModel) -> MotionCommand:
        """状況に応じて実行するtacticを切り替えてrunする."""

        # ロボットの台数が2台未満の場合はplacementを諦める
        if len(world_model.robots.our_visible_robots) < 2:
            return self.tactic_avoid_and_stay.run(world_model)

        nearest_ball_id = world_model.robot_activity.our_robots_by_ball_distance[0]
        nearest_placement_id = world_model.robot_activity.our_robots_by_placement_distance[0]
        next_nearest_placement_id = world_model.robot_activity.our_robots_by_placement_distance[1]

        # ボールがプレースメント位置についたら
        if world_model.ball_activity.ball_is_on_placement_area:
            # その場にとどまる
            return self.tactic_avoid_and_stay.run(world_model)

        # ボールに一番近かったら
        if nearest_ball_id == self.robot_id:
            # サポートロボットが目的地に到着してない場合
            if not world_model.robot_activity.our_robot_arrived(nearest_placement_id):
                # ボールに近づく
                return self.approach_to_ball(world_model)

            # サポートロボットが到着したら、ボールをドリブルする
            return self.dribble_ball(world_model)

        # プレースメント位置に一番近かったら
        if nearest_placement_id == self.robot_id:
            return self.support_placement(world_model)

        # ボールに一番近いロボットがプレースメント位置にも一番近い場合
        # 2番目に近いロボットが対応する
        if nearest_placement_id == nearest_ball_id and next_nearest_placement_id == self.robot_id:
            return self.support_placement(world_model)

        # それ以外の場合は
        # プレースメントエリア回避ONで、その場にとどまる
        return self.tactic_avoid_and_stay.run(world_model)

    def approach_to_ball(self, world_model: WorldModel) -> MotionCommand:
        """ボールに近づくコマンドを返す."""
        # ドリブル目標位置をボールにすることで、ボールに近づくだけの動きになる
        self.tactic_dribble.target_pos = world_model.ball.pos
        command = self.tactic_dribble.run(world_model)
        # ディフェンスエリア内の移動を許可する
        command.navi_options.avoid_defense_area = False
        return command

    def dribble_ball(self, world_model: WorldModel) -> MotionCommand:
        """ボールをドリブルするコマンドを返す."""
        self.tactic_dribble.target_pos = world_model.referee.placement_pos
        command = self.tactic_dribble.run(world_model)
        # ボールをこぼさないように走行速度を落とす
        command.desired_velocity.x = self.DRIBBLE_VELOCITY
        # ディフェンスエリア内の移動を許可する
        command.navi_options.avoid_defense_area = False
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

        # ディフェンスエリア内の移動を許可する
        command.navi_options.avoid_defense_area = False

        # 移動時にボールと衝突しないように回避する
        command.navi_options.avoid_ball = True

        return command
