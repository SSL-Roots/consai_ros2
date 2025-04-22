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
ボールの位置に応じたPlayを定義するモジュール.

ディフェンスエリア内外でのPlayを設定する.
"""

from consai_game.core.play.play import Play, invert_conditions
from consai_game.play.conditions.ball_conditions import BallConditions
from consai_game.play.conditions.referee_conditions import RefereeConditions
from consai_game.tactic.position import Position
from consai_game.tactic.stop import Stop
from consai_game.tactic.kick.shoot import Shoot


def outside_defense_area() -> Play:
    """ボールがディフェンスエリアの外にあるときのPlayを生成する関数."""
    applicable = [
        RefereeConditions.running,
        BallConditions.is_in_our_defense_area.invert(),
        BallConditions.is_in_their_defense_area.invert(),
    ]
    return Play(
        name="Running. Ball is outside defense area",
        description="ボールがディフェンスエリアの外にあるときのPlay",
        applicable=applicable,
        aborted=invert_conditions(applicable),
        timeout_ms=0,
        roles=[
            [Position(-6.0, 0.0)],
            [Shoot()],
            [Position(-3.0, 2.0)],
            [Position(-3.0, 1.0)],
            [Position(-3.0, 0.0)],
            [Position(-3.0, -1.0)],
            [Position(-3.0, -2.0)],
            [Position(-3.0, -3.0)],
            [Position(-2.0, 1.0)],
            [Position(-2.0, 0.0)],
            [Position(-2.0, -1.0)],
        ],
    )


def in_our_defense_area() -> Play:
    """ボールが自チームのディフェンスエリアにあるときのPlayを生成する関数."""
    applicable = [
        RefereeConditions.running,
        BallConditions.is_in_our_defense_area,
    ]
    return Play(
        name="Running. Ball is in our defense area",
        description="ボールが自チームのディフェンスエリアにあるときのPlay",
        applicable=applicable,
        aborted=invert_conditions(applicable),
        timeout_ms=0,
        roles=[
            [Shoot()],
            [Stop()],
            [Stop()],
            [Stop()],
            [Stop()],
            [Stop()],
            [Stop()],
            [Stop()],
            [Stop()],
            [Stop()],
            [Stop()],
        ],
    )


def in_their_defense_area() -> Play:
    """ボールが相手チームのディフェンスエリアにあるときのPlayを生成する関数."""
    applicable = [
        RefereeConditions.running,
        BallConditions.is_in_their_defense_area,
    ]
    return Play(
        name="Running. Ball is in their defense area",
        description="ボールが相手チームのディフェンスエリアにあるときのPlay",
        applicable=applicable,
        aborted=invert_conditions(applicable),
        timeout_ms=0,
        roles=[
            [Stop()],
            [Stop()],
            [Stop()],
            [Stop()],
            [Stop()],
            [Stop()],
            [Stop()],
            [Stop()],
            [Stop()],
            [Stop()],
            [Stop()],
        ],
    )
