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
    blank_plays.our_free_kick(),
    blank_plays.their_free_kick(),
    blank_plays.our_kick_off(),
    blank_plays.their_kick_off(),
    blank_plays.our_penalty_kick(),
    blank_plays.their_penalty_kick(),
    blank_plays.our_goal(),
    blank_plays.their_goal(),
    blank_plays.our_timeout(),
    blank_plays.their_timeout(),
    blank_plays.our_ball_placement(),
    blank_plays.their_ball_placement(),
]
