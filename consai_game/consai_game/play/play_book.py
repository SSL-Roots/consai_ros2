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
from consai_game.play.play_loader import PlayLoader
from dataclasses import dataclass, field
from itertools import chain
from pathlib import Path
from typing import List


@dataclass
class PlayBook:
    """複数のPlayを管理するPlayBook."""

    plays: List[Play] = field(default_factory=list)

    @classmethod
    def load_from_directory(cls, dir_path: str) -> "PlayBook":
        """ディレクトリ内のすべての YAML ファイルをロード."""
        plays = []

        directory = Path(dir_path)
        for yaml_file in chain(directory.rglob("*.yaml"), directory.rglob("*.yml")):
            print(f"Loading playbook from {yaml_file}")
            plays.append(PlayLoader.load_from_yaml(yaml_file))

        return cls(plays=plays)
