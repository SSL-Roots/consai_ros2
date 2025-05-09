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

"""ロールの割り当て手法を定義し, 文字列に応じてインスタンスを生成するモジュール."""

from enum import Enum

from consai_game.core.role_assignment.methods.by_cost import ByCost
from consai_game.core.role_assignment.methods.by_id import ByIDMethod
from consai_game.core.role_assignment.methods.by_visible import ByVisible


class RoleAssignmentMethods(Enum):
    """役割割り当ての方法を列挙する."""

    BY_ID = "by_id"
    BY_VISIBLE = "by_visible"
    BY_COST = "by_cost"


def create_method(name: str):
    """役割割り当ての方法を生成する."""
    if name == RoleAssignmentMethods.BY_ID.value:
        return ByIDMethod()
    elif name == RoleAssignmentMethods.BY_VISIBLE.value:
        return ByVisible()
    elif name == RoleAssignmentMethods.BY_COST.value:
        return ByCost()

    raise ValueError(f"Unknown role assignment method: {name}")
