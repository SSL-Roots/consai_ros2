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
from consai_game.tactic.chase_ball import ChaseBall
from consai_game.tactic.move_to_ball import MoveToBall
from consai_game.tactic.wrapper.forbid_moving_in_placement_area import ForbidMovingInPlacementArea
from consai_game.tactic.wrapper.with_avoid_ball_zone import WithAvoidBallZone
from consai_game.world_model.world_model import WorldModel
from consai_tools.geometry import geometry_tools as tools
from consai_game.tactic.back_dribble import BackDribble
from consai_game.utils.generate_dummy_ball_position import generate_dummy_ball_position

from consai_msgs.msg import MotionCommand
from consai_msgs.msg import State2D

from copy import deepcopy


class CompositeBallPlacement(TacticBase):
    def __init__(self):
        super().__init__()
        self.tactic_dribble = Dribble()
        self.tactic_avoid_area_and_stay = ForbidMovingInPlacementArea(tactic=Stay())
        self.tactic_chase_ball = WithAvoidBallZone(ChaseBall())
        self.tactic_approach_to_ball = MoveToBall(distance=0.15)
        self.tactic_avoid_ball = MoveToBall(distance=0.6)
        self.tactic_back_dribble = BackDribble()

        self.placer_id = None
        self.supporter_id = None

    def reset(self, robot_id: int) -> None:
        """Reset the tactic state for the specified robot."""
        super().reset(robot_id)

        # 所有するTacticも初期化する
        self.tactic_dribble.reset(robot_id)
        self.tactic_avoid_area_and_stay.reset(robot_id)
        self.tactic_chase_ball.reset(robot_id)
        self.tactic_approach_to_ball.reset(robot_id)
        self.tactic_avoid_ball.reset(robot_id)
        self.tactic_back_dribble.reset(robot_id)

        self.placer_id = None
        self.supporter_id = None

    def exit(self):
        super().exit()

        # 所有するTacticもexitする
        self.tactic_dribble.exit()
        self.tactic_avoid_area_and_stay.exit()
        self.tactic_chase_ball.exit()
        self.tactic_approach_to_ball.exit()
        self.tactic_avoid_ball.exit()
        self.tactic_back_dribble.exit()

    def run(self, world_model: WorldModel) -> MotionCommand:
        """状況に応じて実行するtacticを切り替えてrunする."""

        # ロボットの台数が2台未満の場合はplacementを諦める
        if len(world_model.robots.our_visible_robots) < 2:
            return self.tactic_avoid_area_and_stay.run(world_model)

        # 担当者のリセット処理
        if self.placer_id not in world_model.robots.our_visible_robots.keys():
            self.placer_id = None
        if self.supporter_id not in world_model.robots.our_visible_robots.keys():
            self.supporter_id = None

        # 担当者をアサインする
        if self.placer_id is None:
            # ボールに近いロボットを担当者にする
            self.placer_id = world_model.robot_activity.our_robots_by_ball_distance[0]
        if self.supporter_id is None:
            # プレースメント位置に近いロボットを担当者にする
            self.supporter_id = world_model.robot_activity.our_robots_by_placement_distance[0]
            if self.supporter_id == self.placer_id:
                # プレースメント位置に近いロボットがボールに近いロボットと同じ場合は
                # 2番目に近いロボットを担当者にする
                self.supporter_id = world_model.robot_activity.our_robots_by_placement_distance[1]

        # ボールがプレースメント位置についたら
        if world_model.ball_activity.ball_is_on_placement_area:
            if self.robot_id == self.placer_id or self.robot_id == self.supporter_id:
                # ボールを扱うロボットの場合は、ボールからまっすぐ離れる
                return self.tactic_avoid_ball.run(world_model)
            else:
                # それ以外のロボットはプレースメントエリアから離れる
                return self.tactic_avoid_area_and_stay.run(world_model)

        # ボールに一番近かったら
        if self.placer_id == self.robot_id:
            # サポートロボットが目的地に到着してない場合
            if not world_model.robot_activity.our_robot_arrived(self.supporter_id):
                # ボールに近づく
                return self.tactic_approach_to_ball.run(world_model)

            # サポートロボットが到着したら、ボールをドリブルする
            return self.dribble_ball(world_model)

        # プレースメント位置に一番近かったら
        if self.supporter_id == self.robot_id:
            return self.support_placement(world_model)

        # それ以外の場合は
        # プレースメントエリア回避ONで、その場にとどまる
        return self.tactic_avoid_area_and_stay.run(world_model)

    def dribble_ball(self, world_model: WorldModel) -> MotionCommand:
        """ボールをドリブルするコマンドを返す."""
        BACK_DRIBBLE_DISTANCE = 0.5
        AREA_MARGIN = 0.4
        # ボールがフィールド外にあるか
        target_pos = deepcopy(world_model.referee.placement_pos)
        need_back_dribble = False
        dummy_ball_in_field = False
        robot_pos = world_model.robots.our_robots.get(self.robot_id).pos

        # ボールが消えることを想定して、仮想的なボール位置を生成する
        ball_pos = generate_dummy_ball_position(ball=world_model.ball, robot_pos=robot_pos)

        # ダミーボールがフィールド内に入った？
        if (
            abs(ball_pos.x) < world_model.field.length / 2 - AREA_MARGIN
            and abs(ball_pos.y) < world_model.field.width / 2 - AREA_MARGIN
        ):
            dummy_ball_in_field = True
        if world_model.ball_position.is_outside_of_top():
            # ボールがフィールドの上にあったら、ボールの下にドリブルする
            need_back_dribble = True
            target_pos.x = ball_pos.x
            target_pos.y = ball_pos.y - BACK_DRIBBLE_DISTANCE
        elif world_model.ball_position.is_outside_of_bottom():
            # ボールがフィールドの下にあったら、ボールの上にドリブルする
            need_back_dribble = True
            target_pos.x = ball_pos.x
            target_pos.y = ball_pos.y + BACK_DRIBBLE_DISTANCE
        elif world_model.ball_position.is_outside_of_left():
            # ボールがフィールドの左にあったら、ボールの右にドリブルする
            need_back_dribble = True
            target_pos.x = ball_pos.x + BACK_DRIBBLE_DISTANCE
            target_pos.y = ball_pos.y
        elif world_model.ball_position.is_outside_of_right():
            # ボールがフィールドの右にあったら、ボールの左にドリブルする
            need_back_dribble = True
            target_pos.x = ball_pos.x - BACK_DRIBBLE_DISTANCE
            target_pos.y = ball_pos.y

        # 実ボールがフィールド外、かつダミーボールもフィールド外
        if need_back_dribble and not dummy_ball_in_field:
            # ボールがフィールド外にある場合は、バックドリブルする
            self.tactic_back_dribble.target_pos = target_pos
            command = self.tactic_back_dribble.run(world_model)
        else:
            # ボールがフィールド内にある場合は、ドリブルする
            self.tactic_dribble.target_pos = target_pos
            command = self.tactic_dribble.run(world_model)

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
