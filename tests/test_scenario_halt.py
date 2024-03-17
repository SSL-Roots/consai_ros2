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

import math
import time

from rcst.communication import Communication


def test_halt(rcst_comm: Communication):
    rcst_comm.send_empty_world()
    rcst_comm.send_ball(0, 0)
    for i in range(11):
        rcst_comm.send_blue_robot(i, -1.0, 3.0 - i * 0.5, math.radians(0))
    time.sleep(3)  # Wait for the robots to be placed.

    rcst_comm.change_referee_command('STOP', 3.0)  # Robots are moving
    rcst_comm.change_referee_command('HALT', 2.0)  # Robots should stop

    rcst_comm.observer.reset()
    halt_success = True
    rcst_comm.send_ball(0, 0, 5.0, 0.0)  # Move the ball
    for _ in range(3):
        if rcst_comm.observer.robot_speed().some_blue_robots_over(0.01):
            halt_success = False
            break
        time.sleep(1)
    assert halt_success is True
