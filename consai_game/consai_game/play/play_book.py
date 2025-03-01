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
from dataclasses import dataclass, field
import os
from typing import List
import yaml


@dataclass
class PlayBook:
    """複数のPlayを管理するPlayBook"""
    plays: List[Play] = field(default_factory=list)

    @classmethod
    def load_from_directory(cls, dir_path: str) -> "PlayBook":
        """ディレクトリ内のすべての YAML ファイルをロード"""
        plays = []
        for file_name in os.listdir(dir_path):
            if file_name.endswith(".yaml") or file_name.endswith(".yml"):
                file_path = os.path.join(dir_path, file_name)
                with open(file_path, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f)
                    play = Play(**data)
                    plays.append(play)
        return cls(plays=plays)
