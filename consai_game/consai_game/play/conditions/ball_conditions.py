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

from consai_game.world_model.world_model import WorldModel
from consai_game.core.play.play_condition import PlayCondition
from consai_game.utils.geometry import state2d_norm


class BallConditions:
    @staticmethod
    def velocity_is_lower_than(threshold: float) -> PlayCondition:
        def condition(world_model: WorldModel) -> bool:
            velocity = state2d_norm(world_model.ball.vel)
            return velocity < threshold

        return PlayCondition(condition)

    is_in_our_defense_area = PlayCondition(lambda world_model: world_model.ball_position.is_in_our_defense_area())

    is_in_their_defense_area = PlayCondition(lambda world_model: world_model.ball_position.is_in_their_defense_area())
