# Copyright 2024 Roots
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

"""位置と速度を保持するクラスを提供するモジュール."""

from consai_msgs.msg import State2D


class PosVel:
    """位置と速度を保持するクラス."""

    def __init__(self, pos=State2D(), vel=State2D()):
        """位置と速度を設定する初期化関数."""
        self._pos = pos
        self._vel = vel

    def set_pos(self, x: float, y: float, theta: float = 0.0):
        """位置を設定する関数."""
        self._pos = State2D(x=x, y=y, theta=theta)

    def set_vel(self, x: float, y: float, theta: float = 0.0):
        """速度を設定する関数."""
        self._vel = State2D(x=x, y=y, theta=theta)

    def pos(self) -> State2D:
        """位置を返す関数."""
        return self._pos

    def vel(self) -> State2D:
        """速度を返す関数."""
        return self._vel
