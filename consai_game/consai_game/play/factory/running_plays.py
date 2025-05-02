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

ボールが動くか一定時間経過するとrunning状態になり実行される.
ディフェンスエリア内外でのPlayを設定する.
"""

from consai_game.core.play.play import Play, invert_conditions
from consai_game.play.conditions.ball_conditions import BallConditions
from consai_game.play.conditions.referee_conditions import RefereeConditions
from consai_game.tactic.position import Position
from consai_game.tactic.wrapper.allow_move_in_defense_area import AllowMoveInDefenseArea
from consai_game.tactic.composite.composite_goalie import CompositeGoalie
from consai_game.tactic.composite.composite_offense import CompositeOffense
from consai_game.tactic.wrapper.wrapper_look_ball import WrapperLookBall
from consai_game.tactic.composite.composite_defense import CompositeDefense
from consai_game.tactic.center_back import CenterBack
from consai_game.tactic.composite.composite_man_mark import CompositeManMark
from consai_game.tactic.wrapper.move_to_receive_pass import MoveToReceivePass


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
            [AllowMoveInDefenseArea(CompositeGoalie())],
            [CompositeOffense(tactic_default=MoveToReceivePass(Position(3.0, 2.0)))],
            [CompositeOffense(tactic_default=MoveToReceivePass(Position(3.0, 0.0)))],
            [CompositeOffense(tactic_default=MoveToReceivePass(Position(3.0, -2.0)))],
            [CompositeDefense(tactic_default=CenterBack(), do_receive=False)],
            [CompositeDefense(tactic_default=CenterBack(), do_receive=False)],
            [CompositeDefense(tactic_default=CenterBack(), do_receive=False)],
            [CompositeDefense(tactic_default=CompositeManMark(tactic_default=WrapperLookBall(Position(-2.0, -2.0))))],
            [CompositeDefense(tactic_default=CompositeManMark(tactic_default=WrapperLookBall(Position(-2.0, 2.0))))],
            [CompositeDefense(tactic_default=CompositeManMark(tactic_default=WrapperLookBall(Position(-3.0, -4.0))))],
            [CompositeDefense(tactic_default=CompositeManMark(tactic_default=WrapperLookBall(Position(-3.0, 4.0))))],
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
            [AllowMoveInDefenseArea(CompositeGoalie())],
            [CompositeOffense(tactic_default=WrapperLookBall(Position(-3.0, 2.0)))],
            [CompositeOffense(tactic_default=WrapperLookBall(Position(-3.0, 0.0)))],
            [CompositeOffense(tactic_default=WrapperLookBall(Position(-3.0, -2.0)))],
            [CompositeOffense(tactic_default=WrapperLookBall(Position(0.0, 4.0)))],
            [CompositeOffense(tactic_default=WrapperLookBall(Position(0.0, 2.0)))],
            [CompositeOffense(tactic_default=WrapperLookBall(Position(0.0, -2.0)))],
            [CompositeOffense(tactic_default=WrapperLookBall(Position(0.0, -4.0)))],
            [CompositeOffense(tactic_default=WrapperLookBall(Position(3.0, 3.0)))],
            [CompositeOffense(tactic_default=WrapperLookBall(Position(3.0, -3.0)))],
            [CompositeOffense(tactic_default=WrapperLookBall(Position(4.0, 0.0)))],
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
            [AllowMoveInDefenseArea(CompositeGoalie())],
            [CompositeOffense(tactic_default=WrapperLookBall(Position(-3.0, 2.0)))],
            [CompositeOffense(tactic_default=WrapperLookBall(Position(-3.0, 0.0)))],
            [CompositeOffense(tactic_default=WrapperLookBall(Position(-3.0, -2.0)))],
            [CompositeOffense(tactic_default=WrapperLookBall(Position(0.0, 4.0)))],
            [CompositeOffense(tactic_default=WrapperLookBall(Position(0.0, 2.0)))],
            [CompositeOffense(tactic_default=WrapperLookBall(Position(0.0, -2.0)))],
            [CompositeOffense(tactic_default=WrapperLookBall(Position(0.0, -4.0)))],
            [CompositeOffense(tactic_default=WrapperLookBall(Position(3.0, 3.0)))],
            [CompositeOffense(tactic_default=WrapperLookBall(Position(3.0, -3.0)))],
            [CompositeOffense(tactic_default=WrapperLookBall(Position(4.0, 0.0)))],
        ],
    )
