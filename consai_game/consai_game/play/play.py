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

from consai_game.play.applicable_condition import ApplicableCondition
from consai_game.world_model.world_model import WorldModel
from dataclasses import dataclass, field
from typing import List


@dataclass
class Play:
    name: str
    description: str
    applicable: List[ApplicableCondition]
    aborted: List[str]
    timeout_ms: int
    role0: List[str] = field(default_factory=list)
    role1: List[str] = field(default_factory=list)
    role2: List[str] = field(default_factory=list)
    role3: List[str] = field(default_factory=list)
    role4: List[str] = field(default_factory=list)
    role5: List[str] = field(default_factory=list)
    role6: List[str] = field(default_factory=list)
    role7: List[str] = field(default_factory=list)
    role8: List[str] = field(default_factory=list)
    role9: List[str] = field(default_factory=list)
    role10: List[str] = field(default_factory=list)

    def is_applicable(self, world_model: WorldModel) -> bool:
        """Playが適用可能か判定"""
        return all(cond.is_met(world_model) for cond in self.applicable)
