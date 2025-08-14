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
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional

from consai_tools.geometry import geometry_tools as tools

from consai_game.world_model.ball_model import BallModel
from consai_game.world_model.robots_model import Robot, RobotsModel
from consai_game.world_model.referee_model import RefereeModel
from consai_game.world_model.game_config_model import GameConfigModel
from consai_game.world_model.field_model import FieldPoints
from consai_game.world_model.perception.ball_prediction import BallPrediction

from consai_msgs.msg import State2D


class BallDecision:
    """ ボールの位置や動きを判定するクラス. """

    def is_ball_moving(self, ball: BallModel) -> bool:
        """ボールが動いているかを判定するメソッド."""

        # ボールが動いたと判断する距離
        # ここが小さすぎると、ノイズによって動いた判定になってしまう
        BALL_MOVING_DIST_THRESHOLD = 0.3
        MOVING_VELOCITY_THRESHOLD = 0.1  # ボールが動いているとみなす速度の閾値

        if not ball.is_visible:
            return False

        vel_norm = tools.get_norm(ball.vel)

        if vel_norm < MOVING_VELOCITY_THRESHOLD:
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

        if move_distance > BALL_MOVING_DIST_THRESHOLD:
            # 一定距離以上離れたら、動いたと判定してキャプチャした位置をリセット
            self.last_ball_pos_to_detect_moving = None
            return True

        # 一定距離移動してなければ、ボールは止まっていると判断
        return False
