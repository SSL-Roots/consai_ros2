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


from consai_game.core.play.play import Play, invert_conditions
from consai_game.play.conditions.referee_conditions import RefereeConditions
from consai_game.tactic.slow_safe_position import SlowSafePosition
from consai_game.tactic.stop import Stop
from consai_game.tactic.kick.shoot import Shoot


def halt() -> Play:
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
    applicable = [
        RefereeConditions.stop,
    ]
    return Play(
        name="stop",
        description="STOP信号をトリガーにした、デバッグ用の空のPlay",
        applicable=applicable,
        aborted=invert_conditions(applicable),
        timeout_ms=0,
        roles=[
            [SlowSafePosition(-6.0, 0.0)],
            [SlowSafePosition(-3.0, 3.0)],
            [SlowSafePosition(-3.0, 2.0)],
            [SlowSafePosition(-3.0, 1.0)],
            [SlowSafePosition(-3.0, 0.0)],
            [SlowSafePosition(-3.0, -1.0)],
            [SlowSafePosition(-3.0, -2.0)],
            [SlowSafePosition(-3.0, -3.0)],
            [SlowSafePosition(-2.0, 1.0)],
            [SlowSafePosition(-2.0, 0.0)],
            [SlowSafePosition(-2.0, -1.0)],
        ],
    )


def force_start() -> Play:
    applicable = [
        RefereeConditions.force_start,
    ]
    return Play(
        name="force_start",
        description="FORCE_START信号をトリガーにした、デバッグ用の空のPlay",
        applicable=applicable,
        aborted=invert_conditions(applicable),
        timeout_ms=0,
        roles=[
            [Stop()],
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
        ],
    )


def our_free_kick() -> Play:
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
            [Stop()],
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
        ],
    )


def their_free_kick() -> Play:
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


def our_kick_off() -> Play:
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


def their_kick_off() -> Play:
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


def our_kick_off_start() -> Play:
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
            [Stop()],
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
        ],
    )


def their_kick_off_start() -> Play:
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


def our_penalty_kick() -> Play:
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


def their_penalty_kick() -> Play:
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


def our_penalty_kick_start() -> Play:
    applicable = [
        RefereeConditions.our_penalty_kick_start,
    ]
    return Play(
        name="our_penalty_kick_start",
        description="ペナルティキック スタート信号をトリガーにした、デバッグ用の空のPlay",
        applicable=applicable,
        aborted=invert_conditions(applicable),
        timeout_ms=0,
        roles=[
            [Stop()],
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
        ],
    )


def their_penalty_kick_start() -> Play:
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


def our_goal() -> Play:
    applicable = [
        RefereeConditions.our_goal,
    ]
    return Play(
        name="our_goal",
        description="ゴール信号をトリガーにした、デバッグ用の空のPlay",
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


def their_goal() -> Play:
    applicable = [
        RefereeConditions.their_goal,
    ]
    return Play(
        name="their_goal",
        description="ゴール信号をトリガーにした、デバッグ用の空のPlay",
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


def our_timeout() -> Play:
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


def their_timeout() -> Play:
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


def our_ball_placement() -> Play:
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


def their_ball_placement() -> Play:
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
