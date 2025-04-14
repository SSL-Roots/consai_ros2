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

from consai_game.play.factory import simple_plays
from consai_game.play.factory import running_plays

plays = [
    simple_plays.halt(),
    simple_plays.stop(),
    simple_plays.force_start(),
    # simple_plays.running(),
    running_plays.outside_defense_area(),
    running_plays.in_our_defense_area(),
    running_plays.in_their_defense_area(),
    simple_plays.normal_start(),
    simple_plays.our_free_kick(),
    simple_plays.their_free_kick(),
    simple_plays.our_kick_off(),
    simple_plays.their_kick_off(),
    simple_plays.our_penalty_kick(),
    simple_plays.their_penalty_kick(),
    simple_plays.our_goal(),
    simple_plays.their_goal(),
    simple_plays.our_timeout(),
    simple_plays.their_timeout(),
    simple_plays.our_ball_placement(),
    simple_plays.their_ball_placement(),
]
