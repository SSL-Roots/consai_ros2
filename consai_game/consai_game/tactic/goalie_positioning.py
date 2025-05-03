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

    # ロボットの半径[m]
    ROBOT_RADIUS = 0.1

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
        x, y = self.get_goalie_position_free_ball(field, ball)

        command.desired_pose.x = x
        command.desired_pose.y = y
        command.desired_pose.theta = 0.0

        return command

    def get_goalie_position_free_ball(self, field: Field, ball: BallModel):
        """
        ボールを誰も保持していないとき、もしくは味方ロボットが保持しているときのゴーリーのポジションを生成する
        ボールの位置とゴール両端へのベクトルの長さを比較して、短い方のベクトルの方向にゴーリーを配置する
        """

        # 1. ゴールの上端・下端
        goal_x = -field.half_length + self.ROBOT_RADIUS
        goal_y_top = field.half_goal_width
        goal_y_bottom = -field.half_goal_width

        # 2. ボール位置
        ball_x = ball.pos.x
        ball_y = ball.pos.y

        # 3. BGt, BGbベクトルと長さ
        vec_BGt = [goal_x - ball_x, goal_y_top - ball_y]
        vec_BGb = [goal_x - ball_x, goal_y_bottom - ball_y]
        len_BGt = math.hypot(vec_BGt[0], vec_BGt[1])
        len_BGb = math.hypot(vec_BGb[0], vec_BGb[1])

        if len_BGt < len_BGb:
            # BGtが短い場合
            base_len = len_BGt
            # BGb方向の単位ベクトル
            vec_BGb_n = [vec_BGb[0] / len_BGb, vec_BGb[1] / len_BGb]
            # Gb'を計算
            G_other_x = ball_x + vec_BGb_n[0] * base_len
            G_other_y = ball_y + vec_BGb_n[1] * base_len
            # 底辺はGt-Gb'
            base_goal_x = goal_x
            base_goal_y = goal_y_top
        else:
            # BGbが短い場合
            base_len = len_BGb
            # BGt方向の単位ベクトル
            vec_BGt_n = [vec_BGt[0] / len_BGt, vec_BGt[1] / len_BGt]
            # Gt'を計算
            G_other_x = ball_x + vec_BGt_n[0] * base_len
            G_other_y = ball_y + vec_BGt_n[1] * base_len
            # 底辺はGb-Gt'
            base_goal_x = goal_x
            base_goal_y = goal_y_bottom

        # 4. 底辺の中点
        mid_x = (base_goal_x + G_other_x) / 2
        mid_y = (base_goal_y + G_other_y) / 2

        # 5. ゴーリーの配置座標
        return mid_x, mid_y
