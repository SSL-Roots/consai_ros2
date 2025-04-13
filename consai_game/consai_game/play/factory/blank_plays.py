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


from consai_game.core.play.play import Play
from consai_game.play.conditions.ball_conditions import BallConditions
from consai_game.play.conditions.referee_conditions import RefereeConditions
from consai_game.tactic.position import Position
from consai_game.tactic.stop import Stop
from consai_game.tactic.kick.shoot import Shoot


def halt() -> Play:
    return Play(
        name='halt',
        description='HALT信号をトリガーにした、デバッグ用の空のPlay',
        applicable=[
            RefereeConditions.halt,
        ],
        aborted=[
            RefereeConditions.halt.invert(),
        ],
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
            [Stop()]
        ]
    )


def stop() -> Play:
    return Play(
        name='stop',
        description='STOP信号をトリガーにした、デバッグ用の空のPlay',
        applicable=[
            RefereeConditions.stop,
            BallConditions.velocity_is_lower_than(0.1),  # 引数付きのconditionの例
        ],
        aborted=[
            RefereeConditions.stop.invert(),
        ],
        timeout_ms=0,
        roles=[
            [Position(-4.0, 0.0)],
            [Position(-3.0, 3.0)],
            [Position(-3.0, 2.0)],
            [Position(-3.0, 1.0)],
            [Position(-3.0, 0.0)],
            [Position(-3.0, -1.0)],
            [Position(-3.0, -2.0)],
            [Position(-3.0, -3.0)],
            [Position(-2.0, 1.0)],
            [Position(-2.0, 0.0)],
            [Position(-2.0, -1.0)],
        ]
    )


def force_start() -> Play:
    return Play(
        name='force_start',
        description='FORCE_START信号をトリガーにした、デバッグ用の空のPlay',
        applicable=[
            RefereeConditions.force_start,
        ],
        aborted=[
            RefereeConditions.force_start.invert(),
        ],
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
        ]
    )


def normal_start() -> Play:
    return Play(
        name='normal_start',
        description='START信号をトリガーにした、デバッグ用の空のPlay',
        applicable=[
            RefereeConditions.normal_start,
        ],
        aborted=[
            RefereeConditions.normal_start.invert(),
        ],
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
        ]
    )


def our_free_kick() -> Play:
    return Play(
        name='our_free_kick',
        description='フリーキック信号をトリガーにした、デバッグ用の空のPlay',
        applicable=[
            RefereeConditions.our_free_kick,
        ],
        aborted=[
            RefereeConditions.our_free_kick.invert(),
        ],
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
        ]
    )


def their_free_kick() -> Play:
    return Play(
        name='their_free_kick',
        description='フリーキック信号をトリガーにした、デバッグ用の空のPlay',
        applicable=[
            RefereeConditions.their_free_kick,
        ],
        aborted=[
            RefereeConditions.their_free_kick.invert(),
        ],
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
        ]
    )


def our_kick_off() -> Play:
    return Play(
        name='our_kick_off',
        description='キックオフ信号をトリガーにした、デバッグ用の空のPlay',
        applicable=[
            RefereeConditions.our_kick_off,
        ],
        aborted=[
            RefereeConditions.our_kick_off.invert(),
        ],
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
        ]
    )


def their_kick_off() -> Play:
    return Play(
        name='their_kick_off',
        description='キックオフ信号をトリガーにした、デバッグ用の空のPlay',
        applicable=[
            RefereeConditions.their_kick_off,
        ],
        aborted=[
            RefereeConditions.their_kick_off.invert(),
        ],
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
        ]
    )


def our_penalty_kick() -> Play:
    return Play(
        name='our_penalty_kick',
        description='フリーキック信号をトリガーにした、デバッグ用の空のPlay',
        applicable=[
            RefereeConditions.our_penalty_kick,
        ],
        aborted=[
            RefereeConditions.our_penalty_kick.invert(),
        ],
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
        ]
    )


def their_penalty_kick() -> Play:
    return Play(
        name='their_penalty_kick',
        description='フリーキック信号をトリガーにした、デバッグ用の空のPlay',
        applicable=[
            RefereeConditions.their_penalty_kick,
        ],
        aborted=[
            RefereeConditions.their_penalty_kick.invert(),
        ],
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
        ]
    )


def our_goal() -> Play:
    return Play(
        name='our_goal',
        description='ゴール信号をトリガーにした、デバッグ用の空のPlay',
        applicable=[
            RefereeConditions.our_goal,
        ],
        aborted=[
            RefereeConditions.our_goal.invert(),
        ],
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
        ]
    )


def their_goal() -> Play:
    return Play(
        name='their_goal',
        description='ゴール信号をトリガーにした、デバッグ用の空のPlay',
        applicable=[
            RefereeConditions.their_goal,
        ],
        aborted=[
            RefereeConditions.their_goal.invert(),
        ],
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
        ]
    )


def our_timeout() -> Play:
    return Play(
        name='our_timeout',
        description='タイムアウト信号をトリガーにした、デバッグ用の空のPlay',
        applicable=[
            RefereeConditions.our_timeout,
        ],
        aborted=[
            RefereeConditions.our_timeout.invert(),
        ],
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
        ]
    )


def their_timeout() -> Play:
    return Play(
        name='their_timeout',
        description='タイムアウト信号をトリガーにした、デバッグ用の空のPlay',
        applicable=[
            RefereeConditions.their_timeout,
        ],
        aborted=[
            RefereeConditions.their_timeout.invert(),
        ],
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
        ]
    )


def our_ball_placement() -> Play:
    return Play(
        name='our_ball_placement',
        description='ボール配置信号をトリガーにした、デバッグ用の空のPlay',
        applicable=[
            RefereeConditions.our_ball_placement,
        ],
        aborted=[
            RefereeConditions.our_ball_placement.invert(),
        ],
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
        ]
    )


def their_ball_placement() -> Play:
    return Play(
        name='their_ball_placement',
        description='ボール配置信号をトリガーにした、デバッグ用の空のPlay',
        applicable=[
            RefereeConditions.their_ball_placement,
        ],
        aborted=[
            RefereeConditions.their_ball_placement.invert(),
        ],
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
        ]
    )
