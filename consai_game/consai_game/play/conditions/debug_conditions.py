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

"""デバッグ用にTure/Falseを返すplay_condition."""

from consai_game.world_model.world_model import WorldModel
from consai_game.core.play.play_condition import PlayCondition


def TrueConditoin(world_model: WorldModel) -> bool:
    """Tureを返す関数."""
    return True


def FalseConditoin(world_model: WorldModel) -> bool:
    """Falseを返す関数."""
    return False


class DebugConditions:
    """playのdebug用にTrue/Falseを返すだけのクラス."""

    debug_ture = PlayCondition(TrueConditoin)
    debug_false = PlayCondition(FalseConditoin)
