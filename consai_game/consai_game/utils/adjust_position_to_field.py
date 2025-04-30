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
フィールドサイズに合わせて位置を調整するモジュール.
"""

from consai_game.world_model.world_model import WorldModel
from consai_msgs.msg import State2D


class AdjustPosition:
    """フィールドサイズに合わせて位置を調整するクラス."""

    div_a_length = 12.0
    div_a_width = 9.0

    @classmethod
    def adjust(self, x: float, y: float, theta: float, world_model: WorldModel) -> State2D:
        """指定された状態をフィールドサイズに合わせて調整する."""

        adjusted_state = State2D()
        adjusted_state.x = x * world_model.field.length / self.div_a_length
        adjusted_state.y = y * world_model.field.width / self.div_a_width
        adjusted_state.theta = theta

        return adjusted_state
