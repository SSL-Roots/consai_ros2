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
ボールの状態とボール保持者を管理するモジュール.

ボールの状態や移動状況を更新し, ボールを保持しているロボットを追跡する機能を提供する.
"""

from copy import deepcopy

from consai_tools.geometry import geometry_tools as tools

from consai_game.world_model.ball_model import BallModel

import numpy as np

from consai_msgs.msg import State2D

from consai_game.world_model.field_model import Field
from consai_game.world_model.field_model import FieldPoints
from consai_game.world_model.ball_activity_model import BallActivityModel


class BallDecision:
    """ボールの位置や動きを判定するクラス."""

    BALL_MOVING_DIST_THRESHOLD = 0.3  # ボールが動いたと判断する距離. ここが小さすぎるとノイズにより動いた判定になる.
    MOVING_VELOCITY_THRESHOLD = 0.1  # ボールが動いているとみなす速度の閾値
    SIDE_DIST_THRESHOLD = 0.05  # 横方向にどれだけ離れることを許容するか
    THETA_THRESHOLD = 5  # 最低限守るべきロボットの姿勢 deg
    GOAL_WITH_MARGIN = 0.5

    def __init__(self, x=0.0, y=0.0):
        """Initialize the DefendGoal tactic."""
        super().__init__()
        self.target_pos = State2D(x=x, y=y)

    def is_ball_moving(self, ball: BallModel) -> bool:
        """ボールが動いているかを判定するメソッド."""

        if not ball.is_visible:
            return False

        vel_norm = tools.get_norm(ball.vel)

        if vel_norm < self.MOVING_VELOCITY_THRESHOLD:
            # ボールの速度が小さいときは、ボールが停止したと判断して、位置をキャプチャする
            if self.last_ball_pos_to_detect_moving is None:
                self.last_ball_pos_to_detect_moving = deepcopy(ball.pos)
        else:
            # ボールの速度が大きくて、前回の位置情報を持っていないときは
            # 移動が継続していると判断してTrueを返す
            if self.last_ball_pos_to_detect_moving is None:
                return True

        # 停止時のボール位置からの移動距離
        move_distance = tools.get_distance(ball.pos, self.last_ball_pos_to_detect_moving)

        if move_distance > self.BALL_MOVING_DIST_THRESHOLD:
            # 一定距離以上離れたら、動いたと判定してキャプチャした位置をリセット
            self.last_ball_pos_to_detect_moving = None
            return True

        # 一定距離移動してなければ、ボールは止まっていると判断
        return False

    # back_dribble.py
    def ball_is_front(self, ball_pos: State2D, robot_pos: State2D, dist_threshold: float, target_pos: State2D) -> bool:
        """ロボットがボールの前方にいるかどうかを判定する関数."""

        # ボールを中心に、ターゲットを+x軸とした座標系を作る
        trans = tools.Trans(ball_pos, tools.get_angle(ball_pos, target_pos))
        tr_robot_pos = trans.transform(robot_pos)
        tr_robot_theta = trans.transform_angle(robot_pos.theta)

        # ロボットがボールの後ろにいる
        if tr_robot_pos.x < 0:
            return False

        # ボールが正面から離れすぎている
        if tr_robot_pos.x > dist_threshold:
            return False

        # ロボットがボールを見ていない
        if abs(tr_robot_theta) < np.deg2rad(180 - self.THETA_THRESHOLD):
            return False

        # ボールが横方向に離れすぎている
        if abs(tr_robot_pos.y) > self.SIDE_DIST_THRESHOLD:
            return False
        return True

    # composite_goalie.py
    def is_likely_to_score(
        self, field: Field, field_points: FieldPoints, ball: BallModel, ball_activity: BallActivityModel
    ) -> bool:
        """ボールがゴールに入りそうかどうかを判定する関数."""
        goal_y_top = field.half_goal_width
        goal_y_bottom = -field.half_goal_width
        ball_pos = ball.pos
        ball_stop_position = ball_activity.ball_stop_position

        # ゴールの上端・下端の座標
        # マージンを足して少し広く取る
        goal_top_with_margin = State2D(x=-field.half_length, y=goal_y_top + self.GOAL_WITH_MARGIN)
        goal_bottom_with_margin = State2D(x=-field.half_length, y=goal_y_bottom - self.GOAL_WITH_MARGIN)

        # ボール進行方向がゴールに交差するかどうかを判定する
        intersection = tools.get_line_intersection(
            ball_pos, ball_stop_position, goal_top_with_margin, goal_bottom_with_margin
        )

        # ボールがゴールに到達するか
        # 到着点がディフェンスエリア側にありそうかどうか
        if intersection is not None and ball_stop_position.x < field_points.our_defense_area.top_right.x:
            return True
        else:
            return False
