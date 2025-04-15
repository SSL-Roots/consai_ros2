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

from consai_game.core.play.play_condition import PlayCondition
from consai_game.core.tactic.tactic_base import TacticBase
from consai_game.world_model.world_model import WorldModel
from dataclasses import dataclass, field
from typing import List


@dataclass
class Play:
    name: str
    description: str
    applicable: List[PlayCondition]
    aborted: List[PlayCondition]
    timeout_ms: int
    roles: List[List[TacticBase]] = field(default_factory=lambda: [[] for _ in range(11)])

    def is_applicable(self, world_model: WorldModel) -> bool:
        """Playが適用可能か判定."""
        return all(cond.is_met(world_model) for cond in self.applicable)

    def should_abort(self, world_model: WorldModel) -> bool:
        """Playが中断されるべきか判定."""
        return any(cond.is_met(world_model) for cond in self.aborted)


def invert_conditions(conditions: List[PlayCondition]) -> List[PlayCondition]:
    """条件を反転させる."""
    return [cond.invert() for cond in conditions]
