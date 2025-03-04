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

from consai_game.play.books import playbook_default


class PlaybooksLibrarian:
    """Playbook を管理するクラス."""
    _playbooks = {
        "default": playbook_default.plays,
    }

    @classmethod
    def get(cls, playbook_name: str):
        """Playbook を取得する."""
        if playbook_name not in cls._playbooks:
            raise ValueError(f'Playbook "{playbook_name}" is not found')
        return cls._playbooks[playbook_name]

    @classmethod
    def keys(cls) -> list[str]:
        """登録されている Playbook のキー一覧を取得."""
        return list(cls._playbooks.keys())
