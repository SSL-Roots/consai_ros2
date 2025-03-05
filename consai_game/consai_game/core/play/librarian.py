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

from consai_game.core.play.play import Play
from dataclasses import dataclass, field


@dataclass
class PlaybooksLibrarian:
    """Playbookを管理するクラス."""

    playbooks: dict[str, list[Play]] = field(default_factory=dict)

    def register(self, name: str, playbook: list[Play]):
        """Playbookを登録する."""
        self.playbooks[name] = playbook

    def get(self, playbook_name: str):
        """Playbookを取得する."""
        if playbook_name not in self.playbooks:
            raise ValueError(f'Playbook "{playbook_name}" is not found')
        return self.playbooks[playbook_name]

    def keys(self) -> list[str]:
        """登録されているPlaybookのキー一覧を取得."""
        return list(self.playbooks.keys())
