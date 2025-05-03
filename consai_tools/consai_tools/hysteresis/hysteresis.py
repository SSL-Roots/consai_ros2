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


class Hysteresis:
    """ヒステリシスを作るモジュール.

    h = Hysteresis(off_threshold=-0.1, on_threshold=0.1, initial_state=False)
    h.update(0.0)  # False
    h.update(0.1)  # False
    h.update(0.2)  # True
    h.update(0.1)  # True
    h.update(0.0)  # True
    h.update(-0.1) # True
    h.update(-0.2) # False
    h.update(-0.1) # False
    """

    def __init__(self, off_threshold, on_threshold, initial_state: bool = False):
        """初期化."""
        self.on_threshold = on_threshold
        self.off_threshold = off_threshold
        self.state = initial_state

    def update(self, value) -> bool:
        """引数にセットした値を元にヒステリシスを更新."""
        if self.state:
            self.state = value >= self.off_threshold
        else:
            self.state = value > self.on_threshold
        return self.state
