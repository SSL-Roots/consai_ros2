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


def normal_start_condition(world_model: WorldModel) -> bool:
    return world_model.referee.normal_start


def kick_off_blue_condition(world_model: WorldModel) -> bool:
    return world_model.referee.kick_off_blue


def kick_off_yellow_condition(world_model: WorldModel) -> bool:
    return world_model.referee.kick_off_yellow


def free_kick_blue_condition(world_model: WorldModel) -> bool:
    return world_model.referee.free_kick_blue


def free_kick_yellow_condition(world_model: WorldModel) -> bool:
    return world_model.referee.free_kick_yellow


def penalty_kick_blue_condition(world_model: WorldModel) -> bool:
    return world_model.referee.penalty_kick_blue


def penalty_kick_yellow_condition(world_model: WorldModel) -> bool:
    return world_model.referee.penalty_kick_yellow


def goal_blue_condition(world_model: WorldModel) -> bool:
    return world_model.referee.goal_blue


def goal_yellow_condition(world_model: WorldModel) -> bool:
    return world_model.referee.goal_yellow


def timeout_blue_condition(world_model: WorldModel) -> bool:
    return world_model.referee.timeout_blue


def timeout_yellow_condition(world_model: WorldModel) -> bool:
    return world_model.referee.timeout_yellow


def ball_placement_blue_condition(world_model: WorldModel) -> bool:
    return world_model.referee.ball_placement_blue


def ball_placement_yellow_condition(world_model: WorldModel) -> bool:
    return world_model.referee.ball_placement_yellow


class RefereeConditions:
    halt = PlayCondition(halt_condition)
    stop = PlayCondition(stop_condition)
    force_start = PlayCondition(force_start_condition)
    normal_start = PlayCondition(normal_start_condition)
    kick_off_blue = PlayCondition(kick_off_blue_condition)
    kick_off_yellow = PlayCondition(kick_off_yellow_condition)
    free_kick_blue = PlayCondition(free_kick_blue_condition)
    free_kick_yellow = PlayCondition(free_kick_yellow_condition)
    penalty_kick_blue = PlayCondition(penalty_kick_blue_condition)
    penalty_kick_yellow = PlayCondition(penalty_kick_yellow_condition)
    goal_blue = PlayCondition(goal_blue_condition)
    goal_yellow = PlayCondition(goal_yellow_condition)
    timeout_blue = PlayCondition(timeout_blue_condition)
    timeout_yellow = PlayCondition(timeout_yellow_condition)
    ball_placement_blue = PlayCondition(ball_placement_blue_condition)
    ball_placement_yellow = PlayCondition(ball_placement_yellow_condition)
