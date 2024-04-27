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

from rcst.communication import Communication


def test_shoot_at_force_start(rcst_comm: Communication):
    rcst_comm.send_empty_world()
    rcst_comm.send_ball(0, 0)
    for i in range(11):
        rcst_comm.send_blue_robot(i, -3.0, 3.0 - 0.5*i, 0.0)
    rcst_comm.send_blue_robot(2, -0.5, 0.0, math.radians(0))

    rcst_comm.observer.reset()
    rcst_comm.change_referee_command('STOP', 5.0)
    rcst_comm.change_referee_command('FORCE_START', 5.0)

    assert rcst_comm.observer.goal().ball_has_been_in_positive_goal() is True
