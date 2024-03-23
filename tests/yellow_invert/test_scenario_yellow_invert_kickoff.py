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


def test_yellow_invert_our_kickoff(rcst_comm: Communication):
    rcst_comm.send_empty_world()
    rcst_comm.send_ball(0, 0)
    rcst_comm.send_yellow_robot(1, 0.5, 0.0, math.radians(180))

    rcst_comm.observer.reset()
    rcst_comm.change_referee_command('STOP', 3.0)
    rcst_comm.change_referee_command('PREPARE_KICKOFF_YELLOW', 3.0)
    rcst_comm.change_referee_command('NORMAL_START', 5.0)

    assert rcst_comm.observer.goal().ball_has_been_in_negative_goal() is True


def test_their_kickoff(rcst_comm: Communication):
    rcst_comm.send_empty_world()
    rcst_comm.send_ball(0, 0)
    rcst_comm.send_yellow_robot(0, 5.5, 0.0, math.radians(180))
    rcst_comm.send_blue_robot(0, -0.1, 0.0, math.radians(0))

    rcst_comm.observer.reset()
    rcst_comm.change_referee_command('STOP', 3.0)
    rcst_comm.change_referee_command('PREPARE_KICKOFF_BLUE', 3.0)
    rcst_comm.change_referee_command('NORMAL_START', 1.0)

    # Shoot to our goal.
    rcst_comm.send_ball(0, 0, 6.0, 0.5)
    time.sleep(5)

    assert rcst_comm.observer.goal().ball_has_been_in_positive_goal() is False
