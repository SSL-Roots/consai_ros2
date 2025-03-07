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

from consai_game.world_model.world_model import WorldModel
from consai_game.core.play.play_condition import PlayCondition


def halt_condition(world_model: WorldModel) -> bool:
    # TODO: レフェリー信号のHALTを判定する
    print('halt_condition called')
    return False


def stop_condition(world_model: WorldModel) -> bool:
    print('stop_condition called')
    return True


class RefereeConditions:
    halt = PlayCondition(halt_condition)
    stop = PlayCondition(stop_condition)
