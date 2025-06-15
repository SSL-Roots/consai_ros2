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
ゲーム設定情報を保持するモデルの定義.
"""

from dataclasses import dataclass


@dataclass
class GameConfigModel:
    """ゲーム設定情報を保持するクラス."""

    goalie_id: int = 0
    our_team_is_yellow: bool = True
    invert: bool = False
    robot_max_linear_vel: float = 0.0
    robot_max_angular_vel: float = 0.0
    robot_max_linear_accel: float = 0.0
    robot_max_angular_accel: float = 0.0

    gravity: float = 9.81  # 重力加速度
    ball_friction_coeff: float = 0.065  # ボールの摩擦係数

    # キック力パラメータ
    max_kick_power: float = 6.0  # 最大キック力 [m/s]
    max_shoot_speed: float = 5.5  # シュート時の最大速度 [m/s]
    max_pass_speed: float = 4.0  # パス時の最大速度 [m/s]
    min_pass_speed: float = 2.0  # パス時の最小速度 [m/s]
    tapping_kick_power: float = 2.0  # タップキック用のパワー [m/s]
