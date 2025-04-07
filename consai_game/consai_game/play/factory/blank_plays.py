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
