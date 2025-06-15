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

from consai_game.core.tactic.tactic_base import TacticBase
from typing import Mapping
from copy import deepcopy


class CompositeTacticBase(TacticBase):
    """CompositeTacticの基底クラス.

    CompositeTacticは、複数のTacticを切り替えて実行する
    """

    def __init__(self, **tactics: Mapping[str, TacticBase]) -> None:
        """tacticsを登録する."""
        super().__init__()
        self.original_name = deepcopy(self.name)  # 元の名前を保存

        self._tactics_map = {}
        for name, tactic in tactics.items():
            self.add_tactic(name, tactic)

    def add_tactic(self, name: str, tactic: TacticBase) -> None:
        """tacticを属性としてセットする"""
        wrapped_tactic = self.wrap_with_preprocess(tactic)
        setattr(self, name, wrapped_tactic)
        self._tactics_map[name] = wrapped_tactic

    def wrap_with_preprocess(self, tactic: TacticBase) -> TacticBase:
        """tacticのメソッドをラップする"""
        original_run = tactic.run

        def wrapped_run(*args, **kwargs):
            self.before_run(running_tactic_name=tactic.name)
            print(f"wrapped run robot_id: {tactic.robot_id}, tactic: {tactic.name}")
            return original_run(*args, **kwargs)

        # メソッドを差し替え
        tactic.run = wrapped_run
        return tactic

    def before_run(self, running_tactic_name: str) -> None:
        """tacticを実行する前に呼ばれる関数."""
        # 実行しているtacticの名前を取得
        self.name = self.original_name + "." + running_tactic_name

    def reset(self, robot_id: int) -> None:
        """tacticsをリセットする関数."""
        super().reset(robot_id)
        for tactic in self._tactics_map.values():
            print(f"reset tactic: {tactic.name}, robot_id: {robot_id}")
            tactic.reset(robot_id)

    def exit(self):
        """tacticsをexitする関数."""
        super().exit()
        for tactic in self._tactics_map.values():
            tactic.exit()
