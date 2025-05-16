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

"""状態遷移を画像で保存するプログラム."""

import os

from consai_game.tactic.dribble import DribbleStateMachine
from consai_game.tactic.kick import KickStateMachine
from consai_game.tactic.swab import SwabStateMachine
from consai_game.tactic.back_dribble import BackDribbleStateMachine


if __name__ == "__main__":
    # 保存先ディレクトリを作成
    save_dir = "./graph_state_machine"
    os.makedirs(save_dir, exist_ok=True)

    model_dict = {
        "dribble": DribbleStateMachine("robot"),
        "back_dribble": BackDribbleStateMachine("robot"),
        "kick": KickStateMachine("robot"),
        "swab": SwabStateMachine("robot"),
    }

    for (name, model) in model_dict.items():
        model.get_graph().draw(os.path.join(save_dir, name + ".png"), prog="dot", format="png")
