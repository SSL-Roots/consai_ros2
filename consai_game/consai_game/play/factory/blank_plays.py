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

def free_kick_blue() -> Play:
    return Play(
        name='free_kick_blue',
        description='フリーキック信号をトリガーにした、デバッグ用の空のPlay',
        applicable=[
            RefereeConditions.free_kick_blue,
        ],
        aborted=[
            RefereeConditions.free_kick_blue.invert(),
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

def free_kick_yellow() -> Play:
    return Play(
        name='free_kick_yellow',
        description='フリーキック信号をトリガーにした、デバッグ用の空のPlay',
        applicable=[
            RefereeConditions.free_kick_yellow,
        ],
        aborted=[
            RefereeConditions.free_kick_yellow.invert(),
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

def kick_off_blue() -> Play:
    return Play(
        name='kick_off_blue',
        description='キックオフ信号をトリガーにした、デバッグ用の空のPlay',
        applicable=[
            RefereeConditions.kick_off_blue,
        ],
        aborted=[
            RefereeConditions.kick_off_blue.invert(),
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

def kick_off_yellow() -> Play:
    return Play(
        name='kick_off_yellow',
        description='キックオフ信号をトリガーにした、デバッグ用の空のPlay',
        applicable=[
            RefereeConditions.kick_off_yellow,
        ],
        aborted=[
            RefereeConditions.kick_off_yellow.invert(),
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

def penalty_kick_blue() -> Play:
    return Play(
        name='penalty_kick_blue',
        description='フリーキック信号をトリガーにした、デバッグ用の空のPlay',
        applicable=[
            RefereeConditions.penalty_kick_blue,
        ],
        aborted=[
            RefereeConditions.penalty_kick_blue.invert(),
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

def penalty_kick_yellow() -> Play:
    return Play(
        name='penalty_kick_yellow',
        description='フリーキック信号をトリガーにした、デバッグ用の空のPlay',
        applicable=[
            RefereeConditions.penalty_kick_yellow,
        ],
        aborted=[
            RefereeConditions.penalty_kick_yellow.invert(),
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

def goal_blue() -> Play:
    return Play(
        name='goal_blue',
        description='ゴール信号をトリガーにした、デバッグ用の空のPlay',
        applicable=[
            RefereeConditions.goal_blue,
        ],
        aborted=[
            RefereeConditions.goal_blue.invert(),
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

def goal_yellow() -> Play:
    return Play(
        name='goal_yellow',
        description='ゴール信号をトリガーにした、デバッグ用の空のPlay',
        applicable=[
            RefereeConditions.goal_yellow,
        ],
        aborted=[
            RefereeConditions.goal_yellow.invert(),
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

def timeout_blue() -> Play:
    return Play(
        name='timeout_blue',
        description='タイムアウト信号をトリガーにした、デバッグ用の空のPlay',
        applicable=[
            RefereeConditions.timeout_blue,
        ],
        aborted=[
            RefereeConditions.timeout_blue.invert(),
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

def timeout_yellow() -> Play:
    return Play(
        name='timeout_yellow',
        description='タイムアウト信号をトリガーにした、デバッグ用の空のPlay',
        applicable=[
            RefereeConditions.timeout_yellow,
        ],
        aborted=[
            RefereeConditions.timeout_yellow.invert(),
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

def ball_placement_blue() -> Play:
    return Play(
        name='ball_placement_blue',
        description='ボール配置信号をトリガーにした、デバッグ用の空のPlay',
        applicable=[
            RefereeConditions.ball_placement_blue,
        ],
        aborted=[
            RefereeConditions.ball_placement_blue.invert(),
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

def ball_placement_yellow() -> Play:
    return Play(
        name='ball_placement_yellow',
        description='ボール配置信号をトリガーにした、デバッグ用の空のPlay',
        applicable=[
            RefereeConditions.ball_placement_yellow,
        ],
        aborted=[
            RefereeConditions.ball_placement_yellow.invert(),
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