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
DefendGoal Tactic.

ボールの位置と動きに基づいて自チームのゴールを守るためのTacticを定義.
"""

from consai_game.core.tactic.tactic_base import TacticBase
from consai_game.world_model.world_model import WorldModel
from consai_game.world_model.field_model import Field
from consai_game.world_model.ball_model import BallModel
from consai_msgs.msg import MotionCommand

# from consai_msgs.msg import State2D
# from consai_tools.geometry import geometry_tools as tool

import math


class GoaliePositioning(TacticBase):
    """ゴーリーのポジションを生成するTactic"""

    def __init__(self):
        """Initialize the DefendGoal tactic."""
        super().__init__()

    def reset(self, robot_id: int) -> None:
        """Reset the tactic state for the specified robot."""
        super().reset(robot_id)

    def run(self, world_model: WorldModel) -> MotionCommand:
        """Run the tactic and return a MotionCommand based on the ball's position and movement."""
        command = MotionCommand()
        command.robot_id = self.robot_id

        field = world_model.field
        ball = world_model.ball
        # ボールの位置を取得
        # ball_pos = world_model.ball.pos

        # ボールの保持状態を取得
        # ball_holder = world_model.ball_activity.ball_holder

        #########################################################
        # 敵方向の考慮は一旦無視します
        #########################################################

        # if ball_holder is None or ball_holder.is_our_team:
        #     # ボールを誰も保持していないとき、もしくは味方ロボットが保持しているとき
        #     x, y = self.get_goalie_position_free_ball(field, ball)

        # elif not ball_holder.is_our_team:
        #     # ボールを相手ロボットが保持しているとき
        #     robot = ball_holder.robot

        #     # ゴールラインについての座標
        #     l1p1 = world_model.field_points.our_goal_top
        #     l1p2 = world_model.field_points.our_goal_bottom

        #     # 相手ロボットと相手ロボットの向いている線分の座標
        #     l2p1 = robot.pos
        #     l2p2 = State2D()
        #     l2p2.x = robot.pos.x + 10 * math.cos(robot.pos.theta)
        #     l2p2.y = robot.pos.y + 10 * math.sin(robot.pos.theta)

        #     # ロボットの方向とゴールラインの交点
        #     intersecsion = tool.get_line_intersection(
        #         line1_pose1=l1p1, line1_pose2=l1p2, line2_pose1=l2p1, line2_pose2=l2p2, is_on_line1_check=False
        #     )

        #     if intersecsion is None:
        #         # x座標をゴール前にする
        #         x = -world_model.field.half_length + self.ROBOT_RADIUS
        #         # y座標をボールと同じ位置にする
        #         y = ball_pos.y
        #     else:
        #         x = intersecsion.x
        #         y = intersecsion.y

        # # ゴールからはみ出ないようにclamp
        # y = max(min(y, world_model.field.half_goal_width), -world_model.field.half_goal_width)

        # ボールを誰も保持していないとき、もしくは味方ロボットが保持しているとき
        x, y = self._get_goalie_position_free_ball(field, ball, world_model)

        command.desired_pose.x = x
        command.desired_pose.y = y
        command.desired_pose.theta = 0.0

        return command

    def _get_goalie_position_free_ball(self, field: Field, ball: BallModel, world_model: WorldModel):
        """
        ボールを誰も保持していないとき、もしくは味方ロボットが保持しているときのゴーリーのポジションを生成する
        ボールの位置とゴール両端へのベクトルの長さを比較して、短い方のベクトルの方向にゴーリーを配置する
        """

        robot = world_model.robots

        # 1. ゴールの上端・下端
        goal_x = -field.half_length + robot.robot_radius
        goal_y_top = field.half_goal_width
        goal_y_bottom = -field.half_goal_width

        # 2. ボール位置
        ball_x = ball.pos.x
        ball_y = ball.pos.y

        # 3. ball_to_goal_top, ball_to_goal_bottomベクトルと長さ
        vec_ball_to_goal_top = [goal_x - ball_x, goal_y_top - ball_y]
        vec_ball_to_goal_bottom = [goal_x - ball_x, goal_y_bottom - ball_y]
        len_ball_to_goal_top = math.hypot(vec_ball_to_goal_top[0], vec_ball_to_goal_top[1])
        len_ball_to_goal_bottom = math.hypot(vec_ball_to_goal_bottom[0], vec_ball_to_goal_bottom[1])

        if len_ball_to_goal_top < len_ball_to_goal_bottom:
            # ボールが上側にある
            # 短い方に合わせた2等辺三角形を考える
            isosceles_vertex_x, isosceles_vertex_y = self._project_point_by_length(
                ball_x, ball_y, len_ball_to_goal_top, len_ball_to_goal_bottom, vec_ball_to_goal_bottom
            )
            # 底辺はゴール側
            base_goal_x = goal_x
            base_goal_y = goal_y_top
        else:
            # ボールが下側にある
            # 短い方に合わせた2等辺三角形を考える
            isosceles_vertex_x, isosceles_vertex_y = self._project_point_by_length(
                ball_x, ball_y, len_ball_to_goal_bottom, len_ball_to_goal_top, vec_ball_to_goal_top
            )
            # 底辺はゴール側
            base_goal_x = goal_x
            base_goal_y = goal_y_bottom

        # 4. 底辺の中点
        mid_x = (base_goal_x + isosceles_vertex_x) / 2
        mid_y = (base_goal_y + isosceles_vertex_y) / 2

        # 5. ゴーリーの配置座標
        return mid_x, mid_y

    def _project_point_by_length(self, ball_x, ball_y, base_len, longer_len, longer_vec):
        """
        ボールの位置とゴール両端へのベクトルの長さを比較して
        短い方のベクトルの長さに応じて、長い方のベクトルの方向に投影した点を計算する
        つまり短い方に合わせた2等辺三角形を考える
        """
        # 長い方のベクトルの単位ベクトル
        vec_longer_n = [longer_vec[0] / longer_len, longer_vec[1] / longer_len]
        # 2等辺三角形の長い方向側のx,y座標
        isosceles_vertex_x = ball_x + vec_longer_n[0] * base_len
        isosceles_vertex_y = ball_y + vec_longer_n[1] * base_len
        return isosceles_vertex_x, isosceles_vertex_y
