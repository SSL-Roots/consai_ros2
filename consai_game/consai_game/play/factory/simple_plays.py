#!/usr/bin/env python3
# coding: UTF-8

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
デバッグ用の空のPlayを定義するモジュール.

セットプレイ時に, 各信号に対して適切なPlayを作成する関数群.
"""

from consai_game.core.play.play import Play, invert_conditions
from consai_game.play.conditions.referee_conditions import RefereeConditions
from consai_game.tactic.position import Position
from consai_game.tactic.stop import Stop
from consai_game.tactic.composite.chase_or_position import ChaseOrPosition
from consai_game.tactic.composite.composite_goalie import CompositeGoalie
from consai_game.tactic.composite.composite_offense import CompositeOffense
from consai_game.tactic.wrapper.wrapper_look_ball import WrapperLookBall
from consai_game.tactic.wrapper.allow_move_in_defense_area import AllowMoveInDefenseArea
from consai_game.tactic.composite.composite_ball_placement import CompositeBallPlacement
from consai_game.tactic.stay import Stay
from consai_game.tactic.wrapper.forbid_moving_in_placement_area import ForbidMovingInPlacementArea
from consai_game.tactic.wrapper.slow_safe import SlowSafe
from consai_game.tactic.wrapper.with_avoid_ball_zone import WithAvoidBallZone
from consai_game.play.conditions.ball_conditions import BallConditions


def halt() -> Play:
    """HALT信号をトリガーにした, デバッグ用の空のPlayを作成する関数."""
    applicable = [
        RefereeConditions.halt,
    ]
    return Play(
        name="halt",
        description="HALT信号をトリガーにした、デバッグ用の空のPlay",
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


def stop() -> Play:
    """STOP信号をトリガーにした, デバッグ用の空のPlayを作成する関数."""
    applicable = [
        RefereeConditions.stop,
        BallConditions.is_in_their_defense_area.invert(),
    ]
    return Play(
        name="stop",
        description="STOP信号をトリガーにした、デバッグ用の空のPlay",
        applicable=applicable,
        aborted=invert_conditions(applicable),
        timeout_ms=0,
        roles=[
            [AllowMoveInDefenseArea(SlowSafe(WrapperLookBall(Position(-6.0, 0.0))))],
            [SlowSafe(WrapperLookBall(ChaseOrPosition(-3.5, 1.5)))],
            [SlowSafe(WrapperLookBall(ChaseOrPosition(-3.5, 0.5)))],
            [SlowSafe(WrapperLookBall(ChaseOrPosition(-3.5, -0.5)))],
            [SlowSafe(WrapperLookBall(ChaseOrPosition(-3.5, -1.5)))],
            [SlowSafe(WrapperLookBall(ChaseOrPosition(-2.5, 1.1)))],
            [SlowSafe(WrapperLookBall(ChaseOrPosition(-2.5, 0.0)))],
            [SlowSafe(WrapperLookBall(ChaseOrPosition(-2.5, -1.1)))],
            [SlowSafe(WrapperLookBall(ChaseOrPosition(-0.3, 2.5)))],
            [SlowSafe(WrapperLookBall(ChaseOrPosition(-0.7, 0.0)))],
            [SlowSafe(WrapperLookBall(ChaseOrPosition(-0.3, -2.5)))],
        ],
    )


def stop_ball_in_defence_area() -> Play:
    """ボールが相手チームのディフェンスエリアにあるときのPlayを生成する関数."""
    applicable = [
        RefereeConditions.stop,
        BallConditions.is_in_their_defense_area,
    ]
    return Play(
        name="Stop. Ball is in their defense area",
        description="ボールが相手チームのディフェンスエリアにあるときのPlay",
        applicable=applicable,
        aborted=invert_conditions(applicable),
        timeout_ms=0,
        roles=[
            [AllowMoveInDefenseArea(SlowSafe(WrapperLookBall(Position(-6.0, 0.0))))],
            [SlowSafe(WrapperLookBall(Position(-3.0, 2.0)))],
            [SlowSafe(WrapperLookBall(Position(-3.0, 0.0)))],
            [SlowSafe(WrapperLookBall(Position(-3.0, -2.0)))],
            [SlowSafe(WrapperLookBall(Position(0.0, 4.0)))],
            [SlowSafe(WrapperLookBall(Position(0.0, 2.0)))],
            [SlowSafe(WrapperLookBall(Position(0.0, -2.0)))],
            [SlowSafe(WrapperLookBall(Position(0.0, -4.0)))],
            [SlowSafe(WrapperLookBall(Position(3.0, 3.0)))],
            [SlowSafe(WrapperLookBall(Position(3.0, -3.0)))],
            [SlowSafe(WrapperLookBall(Position(3.0, 0.0)))],
        ],
    )


def our_free_kick() -> Play:
    """フリーキック信号をトリガーにした, デバッグ用の空のPlayを作成する関数."""
    applicable = [
        RefereeConditions.our_free_kick,
        RefereeConditions.running.invert(),
    ]
    return Play(
        name="our_free_kick",
        description="フリーキック信号をトリガーにした、デバッグ用の空のPlay",
        applicable=applicable,
        aborted=invert_conditions(applicable),
        timeout_ms=0,
        roles=[
            [AllowMoveInDefenseArea(WrapperLookBall(Position(-6.0, 0.0)))],
            [
                CompositeOffense(
                    is_setplay=True, kick_score_threshold=50, tactic_default=WrapperLookBall(Position(-3.0, 2.0))
                )
            ],
            [
                CompositeOffense(
                    is_setplay=True, kick_score_threshold=50, tactic_default=WrapperLookBall(Position(-3.0, 0.0))
                )
            ],
            [
                CompositeOffense(
                    is_setplay=True, kick_score_threshold=50, tactic_default=WrapperLookBall(Position(-3.0, -2.0))
                )
            ],
            [
                CompositeOffense(
                    is_setplay=True, kick_score_threshold=50, tactic_default=WrapperLookBall(Position(0.0, 4.0))
                )
            ],
            [
                CompositeOffense(
                    is_setplay=True, kick_score_threshold=50, tactic_default=WrapperLookBall(Position(0.0, 2.0))
                )
            ],
            [
                CompositeOffense(
                    is_setplay=True, kick_score_threshold=50, tactic_default=WrapperLookBall(Position(0.0, -2.0))
                )
            ],
            [
                CompositeOffense(
                    is_setplay=True, kick_score_threshold=50, tactic_default=WrapperLookBall(Position(0.0, -4.0))
                )
            ],
            [
                CompositeOffense(
                    is_setplay=True, kick_score_threshold=50, tactic_default=WrapperLookBall(Position(3.0, 3.0))
                )
            ],
            [
                CompositeOffense(
                    is_setplay=True, kick_score_threshold=50, tactic_default=WrapperLookBall(Position(3.0, -3.0))
                )
            ],
            [
                CompositeOffense(
                    is_setplay=True, kick_score_threshold=50, tactic_default=WrapperLookBall(Position(3.0, 1.0))
                )
            ],
        ],
    )


def their_free_kick() -> Play:
    """フリーキック信号をトリガーにした, デバッグ用の空のPlayを作成する関数."""
    applicable = [
        RefereeConditions.their_free_kick,
        RefereeConditions.running.invert(),
    ]
    return Play(
        name="their_free_kick",
        description="フリーキック信号をトリガーにした、デバッグ用の空のPlay",
        applicable=applicable,
        aborted=invert_conditions(applicable),
        timeout_ms=0,
        roles=[
            [AllowMoveInDefenseArea(WrapperLookBall(CompositeGoalie()))],
            [WrapperLookBall(WithAvoidBallZone(ChaseOrPosition(-3.5, 0.5)))],
            [WrapperLookBall(WithAvoidBallZone(ChaseOrPosition(-3.5, 0.0)))],
            [WrapperLookBall(WithAvoidBallZone(ChaseOrPosition(-3.5, -0.5)))],
            [WrapperLookBall(WithAvoidBallZone(ChaseOrPosition(-3.5, 2.7)))],
            [WrapperLookBall(WithAvoidBallZone(ChaseOrPosition(-3.5, 1.5)))],
            [WrapperLookBall(WithAvoidBallZone(ChaseOrPosition(-3.5, -1.5)))],
            [WrapperLookBall(WithAvoidBallZone(ChaseOrPosition(-3.5, -2.7)))],
            [WrapperLookBall(WithAvoidBallZone(ChaseOrPosition(3.0, 3.0)))],
            [WrapperLookBall(WithAvoidBallZone(ChaseOrPosition(3.0, -3.0)))],
            [WrapperLookBall(WithAvoidBallZone(ChaseOrPosition(3.0, 0.0)))],
        ],
    )


def our_kick_off() -> Play:
    """キックオフ信号をトリガーにした, デバッグ用の空のPlayを作成する関数."""
    applicable = [
        RefereeConditions.our_kick_off,
    ]
    return Play(
        name="our_kick_off",
        description="キックオフ信号をトリガーにした、デバッグ用の空のPlay",
        applicable=applicable,
        aborted=invert_conditions(applicable),
        timeout_ms=0,
        roles=[
            [AllowMoveInDefenseArea((WrapperLookBall(Position(-6.0, 0.0))))],
            [WrapperLookBall(ChaseOrPosition(-3.5, 1.5))],
            [WrapperLookBall(ChaseOrPosition(-3.5, 0.5))],
            [WrapperLookBall(ChaseOrPosition(-3.5, -0.5))],
            [WrapperLookBall(ChaseOrPosition(-3.5, -1.5))],
            [WrapperLookBall(ChaseOrPosition(-2.5, 1.1))],
            [WrapperLookBall(ChaseOrPosition(-2.5, 0.3))],
            [WrapperLookBall(ChaseOrPosition(-2.5, -1.1))],
            [WrapperLookBall(ChaseOrPosition(-0.3, 1.0))],
            [WrapperLookBall(ChaseOrPosition(-0.7, 0.0))],
            [WrapperLookBall(ChaseOrPosition(-0.3, -1.0))],
        ],
    )


def their_kick_off() -> Play:
    """キックオフ信号をトリガーにした, デバッグ用の空のPlayを作成する関数."""
    applicable = [
        RefereeConditions.their_kick_off,
    ]
    return Play(
        name="their_kick_off",
        description="キックオフ信号をトリガーにした、デバッグ用の空のPlay",
        applicable=applicable,
        aborted=invert_conditions(applicable),
        timeout_ms=0,
        roles=[
            [AllowMoveInDefenseArea(WrapperLookBall(Position(-6.0, 0.0)))],
            [SlowSafe(ChaseOrPosition(-3.5, 0.2))],
            [SlowSafe(ChaseOrPosition(-3.5, -0.2))],
            [SlowSafe(ChaseOrPosition(-2.5, 1.3))],
            [SlowSafe(ChaseOrPosition(-2.5, 0.5))],
            [SlowSafe(ChaseOrPosition(-2.5, -0.5))],
            [SlowSafe(ChaseOrPosition(-2.5, -1.3))],
            [SlowSafe(ChaseOrPosition(-1.0, 1.8))],
            [SlowSafe(ChaseOrPosition(-1.0, 0.7))],
            [SlowSafe(ChaseOrPosition(-1.0, -0.7))],
            [SlowSafe(ChaseOrPosition(-1.0, -1.8))],
        ],
    )


def our_kick_off_start() -> Play:
    """キックオフ スタート信号をトリガーにした, デバッグ用の空のPlayを作成する関数."""
    applicable = [
        RefereeConditions.our_kick_off_start,
        RefereeConditions.running.invert(),
    ]
    return Play(
        name="our_kick_off_start",
        description="キックオフ スタート信号をトリガーにした、デバッグ用の空のPlay",
        applicable=applicable,
        aborted=invert_conditions(applicable),
        timeout_ms=0,
        roles=[
            [AllowMoveInDefenseArea(WrapperLookBall(Position(-6.0, 0.0)))],
            [CompositeOffense(is_setplay=True, force_pass=False, tactic_default=WrapperLookBall(Position(-3.5, 1.5)))],
            [CompositeOffense(is_setplay=True, force_pass=False, tactic_default=WrapperLookBall(Position(-3.5, 0.5)))],
            [CompositeOffense(is_setplay=True, force_pass=False, tactic_default=WrapperLookBall(Position(-3.5, -0.5)))],
            [CompositeOffense(is_setplay=True, force_pass=False, tactic_default=WrapperLookBall(Position(-3.5, -1.5)))],
            [CompositeOffense(is_setplay=True, force_pass=False, tactic_default=WrapperLookBall(Position(-2.5, 1.1)))],
            [CompositeOffense(is_setplay=True, force_pass=False, tactic_default=WrapperLookBall(Position(-2.5, 0.0)))],
            [CompositeOffense(is_setplay=True, force_pass=False, tactic_default=WrapperLookBall(Position(-2.5, -1.1)))],
            [CompositeOffense(is_setplay=True, force_pass=False, tactic_default=WrapperLookBall(Position(-0.5, 1.0)))],
            [CompositeOffense(is_setplay=True, force_pass=False, tactic_default=WrapperLookBall(Position(-0.5, 0.0)))],
            [CompositeOffense(is_setplay=True, force_pass=False, tactic_default=WrapperLookBall(Position(-0.5, -1.0)))],
        ],
    )


def their_kick_off_start() -> Play:
    """キックオフ スタート信号をトリガーにした, デバッグ用の空のPlayを作成する関数."""
    applicable = [
        RefereeConditions.their_kick_off_start,
        RefereeConditions.running.invert(),
    ]
    return Play(
        name="their_kick_off_start",
        description="キックオフ スタート信号をトリガーにした、デバッグ用の空のPlay",
        applicable=applicable,
        aborted=invert_conditions(applicable),
        timeout_ms=0,
        roles=[
            [AllowMoveInDefenseArea(WrapperLookBall(CompositeGoalie()))],
            [ChaseOrPosition(-3.5, 0.2)],
            [ChaseOrPosition(-3.5, -0.2)],
            [ChaseOrPosition(-3.0, 1.5)],
            [ChaseOrPosition(-2.5, 0.7)],
            [ChaseOrPosition(-2.5, -0.7)],
            [ChaseOrPosition(-3.0, -1.5)],
            [ChaseOrPosition(-2.0, 2.5)],
            [ChaseOrPosition(-1.0, 1.0)],
            [ChaseOrPosition(-1.0, -1.0)],
            [ChaseOrPosition(-2.0, -2.5)],
        ],
    )


def our_penalty_kick() -> Play:
    """ペナルティキック信号をトリガーにした, デバッグ用の空のPlayを作成する関数."""
    applicable = [
        RefereeConditions.our_penalty_kick,
    ]
    return Play(
        name="our_penalty_kick",
        description="ペナルティキック信号をトリガーにした、デバッグ用の空のPlay",
        applicable=applicable,
        aborted=invert_conditions(applicable),
        timeout_ms=0,
        roles=[
            [SlowSafe(ChaseOrPosition(-5.0, 3.7))],
            [SlowSafe(ChaseOrPosition(-5.8, 4.3))],
            [SlowSafe(ChaseOrPosition(-5.4, 4.3))],
            [SlowSafe(ChaseOrPosition(-5.0, 4.3))],
            [SlowSafe(ChaseOrPosition(-4.6, 4.3))],
            [SlowSafe(ChaseOrPosition(-4.2, 4.3))],
            [SlowSafe(ChaseOrPosition(-5.8, 4.0))],
            [SlowSafe(ChaseOrPosition(-5.4, 4.0))],
            [SlowSafe(ChaseOrPosition(-5.0, 4.0))],
            [SlowSafe(ChaseOrPosition(-4.6, 4.0))],
            [SlowSafe(ChaseOrPosition(-4.2, 4.0))],
        ],
    )


def their_penalty_kick() -> Play:
    """ペナルティキック信号をトリガーにした, デバッグ用の空のPlayを作成する関数."""
    applicable = [
        RefereeConditions.their_penalty_kick,
    ]
    return Play(
        name="their_penalty_kick",
        description="ペナルティキック信号をトリガーにした、デバッグ用の空のPlay",
        applicable=applicable,
        aborted=invert_conditions(applicable),
        timeout_ms=0,
        roles=[
            [AllowMoveInDefenseArea(WrapperLookBall(Position(-6.0, 0.0)))],
            [SlowSafe(Position(5.8, 4.3))],
            [SlowSafe(Position(5.4, 4.3))],
            [SlowSafe(Position(5.0, 4.3))],
            [SlowSafe(Position(4.6, 4.3))],
            [SlowSafe(Position(4.2, 4.3))],
            [SlowSafe(Position(5.8, 4.0))],
            [SlowSafe(Position(5.4, 4.0))],
            [SlowSafe(Position(5.0, 4.0))],
            [SlowSafe(Position(4.6, 4.0))],
            [SlowSafe(Position(4.2, 4.0))],
        ],
    )


def our_penalty_kick_start() -> Play:
    """ペナルティキック スタート信号をトリガーにした, デバッグ用の空のPlayを作成する関数."""
    applicable = [
        RefereeConditions.our_penalty_kick_start,
    ]
    return Play(
        name="our_penalty_kick_start",
        description="ペナルティキック スタート信号をトリガーにした、デバッグ用の空のPlay",
        applicable=applicable,
        aborted=invert_conditions(applicable),
        timeout_ms=0,
        # ペナルティキックではインプレイのようにボールを蹴ってほしい
        roles=[
            [
                CompositeOffense(
                    is_setplay=False, tactic_default=WrapperLookBall(Position(-5.0, 3.7)), kick_score_threshold=75
                )
            ],
            [
                CompositeOffense(
                    is_setplay=False, tactic_default=WrapperLookBall(Position(-5.8, 4.3)), kick_score_threshold=75
                )
            ],
            [
                CompositeOffense(
                    is_setplay=False, tactic_default=WrapperLookBall(Position(-5.4, 4.3)), kick_score_threshold=75
                )
            ],
            [
                CompositeOffense(
                    is_setplay=False, tactic_default=WrapperLookBall(Position(-5.0, 4.3)), kick_score_threshold=75
                )
            ],
            [
                CompositeOffense(
                    is_setplay=False, tactic_default=WrapperLookBall(Position(-4.6, 4.3)), kick_score_threshold=75
                )
            ],
            [
                CompositeOffense(
                    is_setplay=False, tactic_default=WrapperLookBall(Position(-4.2, 4.3)), kick_score_threshold=75
                )
            ],
            [
                CompositeOffense(
                    is_setplay=False, tactic_default=WrapperLookBall(Position(-5.8, 4.0)), kick_score_threshold=75
                )
            ],
            [
                CompositeOffense(
                    is_setplay=False, tactic_default=WrapperLookBall(Position(-5.4, 4.0)), kick_score_threshold=75
                )
            ],
            [
                CompositeOffense(
                    is_setplay=False, tactic_default=WrapperLookBall(Position(-5.0, 4.0)), kick_score_threshold=75
                )
            ],
            [
                CompositeOffense(
                    is_setplay=False, tactic_default=WrapperLookBall(Position(-4.6, 4.0)), kick_score_threshold=75
                )
            ],
            [
                CompositeOffense(
                    is_setplay=False, tactic_default=WrapperLookBall(Position(-4.2, 4.0)), kick_score_threshold=75
                )
            ],
        ],
    )


def their_penalty_kick_start() -> Play:
    """ペナルティキック スタート信号をトリガーにした, デバッグ用の空のPlayを生成する関数."""
    applicable = [
        RefereeConditions.their_penalty_kick_start,
    ]
    return Play(
        name="their_penalty_kick_start",
        description="ペナルティキック スタート信号をトリガーにした、デバッグ用の空のPlay",
        applicable=applicable,
        aborted=invert_conditions(applicable),
        timeout_ms=0,
        roles=[
            [AllowMoveInDefenseArea(WrapperLookBall(CompositeGoalie()))],
            [Position(5.8, 4.3)],
            [Position(5.4, 4.3)],
            [Position(5.0, 4.3)],
            [Position(4.6, 4.3)],
            [Position(4.2, 4.3)],
            [Position(5.8, 4.0)],
            [Position(5.4, 4.0)],
            [Position(5.0, 4.0)],
            [Position(4.6, 4.0)],
            [Position(4.2, 4.0)],
        ],
    )


def our_timeout() -> Play:
    """タイムアウト信号をトリガーにした, デバッグ用の空のPlayを生成する関数."""
    applicable = [
        RefereeConditions.our_timeout,
    ]
    return Play(
        name="our_timeout",
        description="タイムアウト信号をトリガーにした、デバッグ用の空のPlay",
        applicable=applicable,
        aborted=invert_conditions(applicable),
        timeout_ms=0,
        roles=[
            [AllowMoveInDefenseArea(WrapperLookBall(Position(-4.0, 0.0)))],
            [AllowMoveInDefenseArea(WrapperLookBall(Position(-3.0, 0.5)))],
            [AllowMoveInDefenseArea(WrapperLookBall(Position(-3.0, -0.5)))],
            [AllowMoveInDefenseArea(WrapperLookBall(Position(-2.0, 1.0)))],
            [AllowMoveInDefenseArea(WrapperLookBall(Position(-2.0, 0.0)))],
            [AllowMoveInDefenseArea(WrapperLookBall(Position(-2.0, -1.0)))],
            [AllowMoveInDefenseArea(WrapperLookBall(Position(-0.5, 2.3)))],
            [AllowMoveInDefenseArea(WrapperLookBall(Position(-1.0, 1.6)))],
            [AllowMoveInDefenseArea(WrapperLookBall(Position(-1.0, 0.0)))],
            [AllowMoveInDefenseArea(WrapperLookBall(Position(-1.0, -1.6)))],
            [AllowMoveInDefenseArea(WrapperLookBall(Position(-0.5, -2.3)))],
        ],
    )


def their_timeout() -> Play:
    """タイムアウト信号をトリガーにした, デバッグ用の空のPlayを生成する関数."""
    applicable = [
        RefereeConditions.their_timeout,
    ]
    return Play(
        name="their_timeout",
        description="タイムアウト信号をトリガーにした、デバッグ用の空のPlay",
        applicable=applicable,
        aborted=invert_conditions(applicable),
        timeout_ms=0,
        roles=[
            [AllowMoveInDefenseArea(WrapperLookBall(Position(-4.0, 0.0)))],
            [AllowMoveInDefenseArea(WrapperLookBall(Position(-3.0, 0.5)))],
            [AllowMoveInDefenseArea(WrapperLookBall(Position(-3.0, -0.5)))],
            [AllowMoveInDefenseArea(WrapperLookBall(Position(-2.0, 1.0)))],
            [AllowMoveInDefenseArea(WrapperLookBall(Position(-2.0, 0.0)))],
            [AllowMoveInDefenseArea(WrapperLookBall(Position(-2.0, -1.0)))],
            [AllowMoveInDefenseArea(WrapperLookBall(Position(-0.5, 2.3)))],
            [AllowMoveInDefenseArea(WrapperLookBall(Position(-1.0, 1.6)))],
            [AllowMoveInDefenseArea(WrapperLookBall(Position(-1.0, 0.0)))],
            [AllowMoveInDefenseArea(WrapperLookBall(Position(-1.0, -1.6)))],
            [AllowMoveInDefenseArea(WrapperLookBall(Position(-0.5, -2.3)))],
        ],
    )


def our_ball_placement() -> Play:
    """ボール配置信号をトリガーにした, デバッグ用の空のPlayを生成する関数."""
    applicable = [
        RefereeConditions.our_ball_placement,
    ]
    return Play(
        name="our_ball_placement",
        description="ボール配置信号をトリガーにした、デバッグ用の空のPlay",
        applicable=applicable,
        aborted=invert_conditions(applicable),
        timeout_ms=0,
        roles=[
            [AllowMoveInDefenseArea(CompositeBallPlacement())],
            [CompositeBallPlacement()],
            [CompositeBallPlacement()],
            [CompositeBallPlacement()],
            [CompositeBallPlacement()],
            [CompositeBallPlacement()],
            [CompositeBallPlacement()],
            [CompositeBallPlacement()],
            [CompositeBallPlacement()],
            [CompositeBallPlacement()],
            [CompositeBallPlacement()],
        ],
    )


def their_ball_placement() -> Play:
    """ボール配置信号をトリガーにした, デバッグ用の空のPlayを生成する関数."""
    applicable = [
        RefereeConditions.their_ball_placement,
    ]
    return Play(
        name="their_ball_placement",
        description="ボール配置信号をトリガーにした、デバッグ用の空のPlay",
        applicable=applicable,
        aborted=invert_conditions(applicable),
        timeout_ms=0,
        roles=[
            [AllowMoveInDefenseArea(ForbidMovingInPlacementArea(Stay()))],
            [ForbidMovingInPlacementArea(Stay())],
            [ForbidMovingInPlacementArea(Stay())],
            [ForbidMovingInPlacementArea(Stay())],
            [ForbidMovingInPlacementArea(Stay())],
            [ForbidMovingInPlacementArea(Stay())],
            [ForbidMovingInPlacementArea(Stay())],
            [ForbidMovingInPlacementArea(Stay())],
            [ForbidMovingInPlacementArea(Stay())],
            [ForbidMovingInPlacementArea(Stay())],
            [ForbidMovingInPlacementArea(Stay())],
        ],
    )
