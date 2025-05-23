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


"""円軌道で移動するPlayを定義するモジュール."""

from consai_game.core.play.play import Play, invert_conditions
from consai_game.play.conditions.debug_conditions import DebugConditions
from consai_game.tactic.wrapper.wrapper_look_ball import WrapperLookBall
from consai_game.tactic.composite.composite_ball_centric_mover import BallCentricMover


def circular_move() -> Play:
    """円軌道で移動するPlayを作成する関数."""
    applicable = [
        DebugConditions.debug_ture,
    ]
    return Play(
        name="circular_move",
        description="ボール中心に円運動をするPlay",
        applicable=applicable,
        aborted=invert_conditions(applicable),
        timeout_ms=0,
        roles=[
            [WrapperLookBall(BallCentricMover(radius=0.3, seconds=20, cw=True))],
            [WrapperLookBall(BallCentricMover(radius=0.6, seconds=20, cw=True))],
            [WrapperLookBall(BallCentricMover(radius=0.9, seconds=20, cw=True))],
            [WrapperLookBall(BallCentricMover(radius=1.2, seconds=20, cw=True))],
            [WrapperLookBall(BallCentricMover(radius=1.5, seconds=20, cw=True))],
            [WrapperLookBall(BallCentricMover(radius=1.8, seconds=20, cw=True))],
            [WrapperLookBall(BallCentricMover(radius=2.1, seconds=20, cw=True))],
            [WrapperLookBall(BallCentricMover(radius=2.4, seconds=20, cw=True))],
            [WrapperLookBall(BallCentricMover(radius=2.7, seconds=20, cw=True))],
            [WrapperLookBall(BallCentricMover(radius=3.0, seconds=20, cw=True))],
            [WrapperLookBall(BallCentricMover(radius=3.3, seconds=20, cw=True))],
        ],
    )
