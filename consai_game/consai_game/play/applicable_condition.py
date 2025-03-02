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
from dataclasses import dataclass

from typing import Callable


@dataclass
class ApplicableCondition:
    """Play が適用可能かを判定する条件."""

    condition: str  # 条件名 (例: "halt", "stop")
    check_func: Callable[[WorldModel], bool]  # 条件を満たしているか判定する関数

    def is_met(self, game_state: WorldModel) -> bool:
        """ゲームの状態を渡して条件を満たしているかチェック."""
        return self.check_func(game_state)
