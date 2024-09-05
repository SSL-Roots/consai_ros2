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

import time
from typing import Literal


def init_placement(rcst_comm, color: Literal["blue", "yellow"],
                    target_x: float, target_y: float,
                    num_of_robots: int = 11,
                    ball_x: float = 0.0, ball_y: float = 0.0):
    rcst_comm.send_empty_world()
    rcst_comm.send_ball(ball_x, ball_y)

    for i in range(num_of_robots):
        rcst_comm.send_blue_robot(i, -1.0, 3.0 - 0.5*i, 0.0)
    time.sleep(3)  # Wait for the robots to be placed.

    for_blue_team = color == "blue"

    rcst_comm.observer.ball_placement().set_targets(
        target_x, target_y, for_blue_team=for_blue_team)
    rcst_comm.set_ball_placement_position(target_x, target_y)

    rcst_comm.change_referee_command('STOP', 1.0)
    if for_blue_team:
        rcst_comm.change_referee_command('BALL_PLACEMENT_BLUE', 0.0)
    else:
        rcst_comm.change_referee_command('BALL_PLACEMENT_YELLOW', 0.0)


def wait_for_placement(rcst_comm, timeout=30):
    placement_success = False
    for _ in range(timeout):
        if rcst_comm.observer.ball_placement().success():
            placement_success = True
            break
        time.sleep(1)
    return placement_success
