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
from consai_game.play.conditions.referee_conditions import RefereeConditions


def halt() -> Play:
    return Play(
        name='halt',
        description='HALT信号をトリガーにした、デバッグ用の空のPlay',
        applicable=[
            RefereeConditions.halt,
        ],
        aborted=['!applicable'],
        timeout_ms=0,
        role0=['blank'],
        role1=['blank'],
        role2=['blank'],
        role3=['blank'],
        role4=['blank'],
        role5=['blank'],
        role6=['blank'],
        role7=['blank'],
        role8=['blank'],
        role9=['blank'],
        role10=['blank']
    )


def stop() -> Play:
    return Play(
        name='stop',
        description='STOP信号をトリガーにした、デバッグ用の空のPlay',
        applicable=[
            RefereeConditions.stop,
        ],
        aborted=['!applicable'],
        timeout_ms=0,
        role0=['blank'],
        role1=['blank'],
        role2=['blank'],
        role3=['blank'],
        role4=['blank'],
        role5=['blank'],
        role6=['blank'],
        role7=['blank'],
        role8=['blank'],
        role9=['blank'],
        role10=['blank']
    )
