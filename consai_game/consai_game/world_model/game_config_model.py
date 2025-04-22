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
ゲーム設定情報を保持するモデルの定義.

チームカラーとゴールキーパーIDに関する設定の管理.
"""

from dataclasses import dataclass


@dataclass
class GameConfigModel:
    """ゲーム設定情報を保持するクラス."""

    goalie_id: int = 0
    our_team_is_yellow: bool = True
