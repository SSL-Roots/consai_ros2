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
ロールに関する定義を含むモジュール.

ロールに関連する定数や, 戦術とロボットIDを保持するRoleクラスを提供する.
"""

from dataclasses import dataclass, field
from typing import Final, List

from consai_game.core.tactic.tactic_base import TacticBase


class RoleConst:
    """ロールに関する定数を管理するクラス."""

    INVALID_ROLE_ID: Final[int] = -1
    MIN_VALID_ROLE_ID: Final[int] = 0
    MAX_VALID_ROLE_ID: Final[int] = 15


@dataclass
class Role:
    """ロールを定義するクラス."""

    tactics: List[TacticBase] = field(default_factory=list)
    robot_id: int = RoleConst.INVALID_ROLE_ID
