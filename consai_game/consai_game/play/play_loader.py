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

from consai_game.play.play import Play
from consai_game.play.applicable_condition import ApplicableCondition
from consai_game.play.referee_conditions import CONDITION_MAP_REFEREE
from pathlib import Path
import yaml

# すべてのCONDITION_MAPを統合
CONDITION_MAP = {
    **CONDITION_MAP_REFEREE,
}


class PlayLoader:
    """YAMLファイルから Play データを読み込むクラス."""

    @staticmethod
    def load_from_yaml(yaml_file: str) -> Play:
        """YAMLファイルを読み込み、Playオブジェクトを作成."""
        with open(yaml_file, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)

        # `applicable` の文字列リストを `ApplicableCondition` に変換
        applicable_conditions = []
        for cond in data.get("applicable", []):
            if cond in CONDITION_MAP:
                applicable_conditions.append(
                    ApplicableCondition(cond, CONDITION_MAP[cond])
                )
            else:
                # 数値条件の場合は `parse_numeric_condition` で変換
                pass
                # numeric_condition = parse_numeric_condition(cond)
                # if numeric_condition:
                #     applicable_conditions.append(numeric_condition)

        return Play(
            name=Path(yaml_file).stem,
            description=data["description"],
            applicable=applicable_conditions,
            aborted=data["aborted"],
            timeout_ms=data["timeout_ms"],
            role0=data.get("role0", []),
            role1=data.get("role1", []),
            role2=data.get("role2", []),
            role3=data.get("role3", []),
            role4=data.get("role4", []),
            role5=data.get("role5", []),
            role6=data.get("role6", []),
            role7=data.get("role7", []),
            role8=data.get("role8", []),
            role9=data.get("role9", []),
            role10=data.get("role10", []),
        )
