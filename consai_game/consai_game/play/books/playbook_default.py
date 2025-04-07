#!/usr/bin/env python3
# coding: UTF-8

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

from consai_game.play.factory import blank_plays

plays = [
    blank_plays.halt(),
    blank_plays.stop(),
    blank_plays.force_start(),
    blank_plays.normal_start(),
    blank_plays.free_kick_blue(),
    blank_plays.free_kick_yellow(),
    blank_plays.kick_off_blue(),
    blank_plays.kick_off_yellow(),
    blank_plays.penalty_kick_blue(),
    blank_plays.penalty_kick_yellow(),
    blank_plays.goal_blue(),
    blank_plays.goal_yellow(),
    blank_plays.timeout_blue(),
    blank_plays.timeout_yellow(),
    blank_plays.ball_placement_blue(),
    blank_plays.ball_placement_yellow(),
]
