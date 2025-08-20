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


"""相対的な位置関係を評価するモジュール."""

import numpy as np

from consai_msgs.msg import State2D

from consai_tools.geometry import geometry_tools as tools


class RelativePositionEvaluation:
    """相対的な位置関係を評価するクラス."""

    def is_robot_backside(
        self, robot_pos: State2D, ball_pos: State2D, target_pos: State2D, angle_ball_to_robot_threshold: int
    ) -> bool:
        """ボールからターゲットを見て、ロボットが後側に居るかを判定するメソッド."""

        # ボールからターゲットへの座標系を作成
        trans = tools.Trans(ball_pos, tools.get_angle(ball_pos, target_pos))
        tr_robot_pos = trans.transform(robot_pos)

        # ボールから見たロボットの位置の角度
        # ボールの後方にいれば角度は90度以上
        tr_ball_to_robot_angle = tools.get_angle(State2D(x=0.0, y=0.0), tr_robot_pos)

        if abs(tr_ball_to_robot_angle) > np.deg2rad(angle_ball_to_robot_threshold):
            return True
        return False

    def is_robot_on_kick_line(
        self, robot_pos: State2D, ball_pos: State2D, target_pos: State2D, width_threshold: float
    ) -> bool:
        """ボールからターゲットまでの直線上にロボットが居るかを判定するメソッド.

        ターゲットまでの距離が遠いと、角度だけで狙いを定めるのは難しいため、位置を使って判定する.
        """

        minimal_theta_threshold = 45  # 最低限満たすべきロボットの角度

        # ボールからターゲットへの座標系を作成
        trans = tools.Trans(ball_pos, tools.get_angle(ball_pos, target_pos))
        tr_robot_pos = trans.transform(robot_pos)
        tr_robot_theta = trans.transform_angle(robot_pos.theta)

        # ボールより前にロボットが居る場合
        if tr_robot_pos.x > 0.0:
            return False

        # ターゲットを向いていない
        if abs(tr_robot_theta) > np.deg2rad(minimal_theta_threshold):
            return False

        if abs(tr_robot_pos.y) > width_threshold:
            return False

        return True

    def is_ball_front(self, robot_pos: State2D, ball_pos: State2D, target_pos: State2D) -> bool:
        """ボールがロボットの前にあるかどうかを判定するメソッド."""

        front_dist_threshold = 0.15  # 正面方向にどれだけ離れることを許容するか
        side_dist_threshold = 0.05  # 横方向にどれだけ離れることを許容するか

        # ロボットを中心に、ターゲットを+x軸とした座標系を作る
        trans = tools.Trans(robot_pos, tools.get_angle(robot_pos, target_pos))
        tr_ball_pos = trans.transform(ball_pos)

        # ボールがロボットの後ろにある
        if tr_ball_pos.x < 0:
            return False

        # ボールが正面から離れすぎている
        if tr_ball_pos.x > front_dist_threshold:
            return False

        # ボールが横方向に離れすぎている
        if abs(tr_ball_pos.y) > side_dist_threshold:
            return False
        return True
