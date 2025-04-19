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


def halt_condition(world_model: WorldModel) -> bool:
    return world_model.referee.halt


def stop_condition(world_model: WorldModel) -> bool:
    return world_model.referee.stop


def force_start_condition(world_model: WorldModel) -> bool:
    return world_model.referee.force_start


def running_condition(world_model: WorldModel) -> bool:
    return world_model.referee.running


def normal_start_condition(world_model: WorldModel) -> bool:
    return world_model.referee.normal_start


def our_kick_off_condition(world_model: WorldModel) -> bool:
    return world_model.referee.our_kick_off


def our_kick_off_start_condition(world_model: WorldModel) -> bool:
    return world_model.referee.our_kick_off_start


def their_kick_off_condition(world_model: WorldModel) -> bool:
    return world_model.referee.their_kick_off


def their_kick_off_start_condition(world_model: WorldModel) -> bool:
    return world_model.referee.their_kick_off_start


def our_free_kick_condition(world_model: WorldModel) -> bool:
    return world_model.referee.our_free_kick


def their_free_kick_condition(world_model: WorldModel) -> bool:
    return world_model.referee.their_free_kick


def our_penalty_kick_condition(world_model: WorldModel) -> bool:
    return world_model.referee.our_penalty_kick


def our_penalty_kick_start_condition(world_model: WorldModel) -> bool:
    return world_model.referee.our_penalty_kick_start


def their_penalty_kick_condition(world_model: WorldModel) -> bool:
    return world_model.referee.their_penalty_kick


def their_penalty_kick_start_condition(world_model: WorldModel) -> bool:
    return world_model.referee.their_penalty_kick_start


def our_goal_condition(world_model: WorldModel) -> bool:
    return world_model.referee.our_goal


def their_goal_condition(world_model: WorldModel) -> bool:
    return world_model.referee.their_goal


def our_timeout_condition(world_model: WorldModel) -> bool:
    return world_model.referee.our_timeout


def their_timeout_condition(world_model: WorldModel) -> bool:
    return world_model.referee.their_timeout


def our_ball_placement_condition(world_model: WorldModel) -> bool:
    return world_model.referee.our_ball_placement


def their_ball_placement_condition(world_model: WorldModel) -> bool:
    return world_model.referee.their_ball_placement


class RefereeConditions:
    halt = PlayCondition(halt_condition)
    stop = PlayCondition(stop_condition)
    force_start = PlayCondition(force_start_condition)
    running = PlayCondition(running_condition)
    normal_start = PlayCondition(normal_start_condition)
    our_kick_off = PlayCondition(our_kick_off_condition)
    their_kick_off = PlayCondition(their_kick_off_condition)
    our_kick_off_start = PlayCondition(our_kick_off_start_condition)
    their_kick_off_start = PlayCondition(their_kick_off_start_condition)
    our_free_kick = PlayCondition(our_free_kick_condition)
    their_free_kick = PlayCondition(their_free_kick_condition)
    our_penalty_kick = PlayCondition(our_penalty_kick_condition)
    their_penalty_kick = PlayCondition(their_penalty_kick_condition)
    our_penalty_kick_start = PlayCondition(our_penalty_kick_start_condition)
    their_penalty_kick_start = PlayCondition(their_penalty_kick_start_condition)
    our_goal = PlayCondition(our_goal_condition)
    their_goal = PlayCondition(their_goal_condition)
    our_timeout = PlayCondition(our_timeout_condition)
    their_timeout = PlayCondition(their_timeout_condition)
    our_ball_placement = PlayCondition(our_ball_placement_condition)
    their_ball_placement = PlayCondition(their_ball_placement_condition)
