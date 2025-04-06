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

from abc import ABC, abstractmethod
from consai_game.world_model.world_model import WorldModel
from consai_msgs.msg import MotionCommand
from enum import Enum, auto


class TacticState(Enum):
    BEFORE_INIT = 0
    RUNNING = auto()
    FINISHED = auto()


class TacticBase(ABC):
    def __init__(self):
        self._robot_id = -1
        self._state = TacticState.BEFORE_INIT

    @abstractmethod
    def reset(self, robot_id: int) -> None:
        raise NotImplementedError()

    @abstractmethod
    def run(self, world_model: WorldModel) -> MotionCommand:
        raise NotImplementedError()

    @property
    def robot_id(self) -> int:
        return self._robot_id

    @robot_id.setter
    def robot_id(self, value: int) -> None:
        self._robot_id = value

    @property
    def state(self) -> TacticState:
        return self._state

    @state.setter
    def state(self, value: TacticState) -> None:
        self._state = value
