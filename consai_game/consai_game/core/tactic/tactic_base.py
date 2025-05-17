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
ロボットの戦術を定義するベースクラス.

戦術の状態管理やロボットIDの管理を行う.
"""

from abc import ABC, abstractmethod

from consai_game.core.tactic.tactic_state import TacticState
from consai_game.world_model.world_model import WorldModel

from consai_msgs.msg import MotionCommand


class TacticBase(ABC):
    """戦術の基本クラス."""

    def __init__(self):
        """TacticBaseの初期化を行う関数."""
        self._robot_id = -1
        self._state = TacticState.BEFORE_INIT
        self._name = self.__class__.__name__

    @abstractmethod
    def run(self, world_model: WorldModel) -> MotionCommand:
        """戦術の実行を行い、MotionCommandを返す関数."""
        raise NotImplementedError()

    def reset(self, robot_id: int) -> None:
        """ロボットIDを設定し、Tacticの状態をRUNNINGにリセットする関数."""
        self.robot_id = robot_id
        self.state = TacticState.RUNNING

    def exit(self) -> None:
        """戦術の状態をFINISHEDにリセットする関数."""
        self.state = TacticState.FINISHED

    @property
    def robot_id(self) -> int:
        """ロボットのIDを取得する関数."""
        return self._robot_id

    @robot_id.setter
    def robot_id(self, value: int) -> None:
        """ロボットのIDを設定する関数."""
        self._robot_id = value

    @property
    def state(self) -> TacticState:
        """戦術の状態を取得する関数."""
        return self._state

    @state.setter
    def state(self, value: TacticState) -> None:
        """戦術の状態を設定する関数."""
        self._state = value

    @property
    def name(self) -> str:
        """戦術の名前を取得する関数."""
        return self._name
