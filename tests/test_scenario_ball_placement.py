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

from utils_for_placement import init_placement, wait_for_placement


def test_near_position(rcst_comm):
    init_placement(rcst_comm=rcst_comm, color="blue", target_x=1.0, target_y=1.0)
    assert wait_for_placement(rcst_comm) is True


def test_far_position(rcst_comm):
    init_placement(rcst_comm=rcst_comm, color="blue", target_x=5.8, target_y=4.3)
    assert wait_for_placement(rcst_comm) is True


def test_so_far_position(rcst_comm):
    init_placement(rcst_comm=rcst_comm, color="blue", target_x=-5.8, target_y=-4.3, ball_x=5.9, ball_y=4.4)
    assert wait_for_placement(rcst_comm) is True
