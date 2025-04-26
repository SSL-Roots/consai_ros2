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
雑巾がけのPlayを定義するモジュール.

Game controllerの信号をもとに動作する
"""

from consai_game.core.play.play import Play, invert_conditions
from consai_game.play.conditions.referee_conditions import RefereeConditions
from consai_game.tactic.stop import Stop
from consai_game.tactic.swab import Swab


def halt() -> Play:
    """HALT信号をトリガーにし停止のPlayを作成する関数."""
    applicable = [
        RefereeConditions.halt,
    ]
    return Play(
        name="halt",
        description="HALT信号をトリガーにした停止用のPlay",
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


def swab() -> Play:
    """STOP信号をトリガーにして雑巾がけのPlayを作成する関数."""
    applicable = [
        RefereeConditions.stop,
    ]
    return Play(
        name="swab",
        description="雑巾がけのPlay",
        applicable=applicable,
        aborted=invert_conditions(applicable),
        timeout_ms=0,
        roles=[
            [Swab()],
            [Swab()],
            [Swab()],
            [Swab()],
            [Swab()],
            [Swab()],
            [Swab()],
            [Swab()],
            [Swab()],
            [Swab()],
            [Swab()],
        ],
    )
