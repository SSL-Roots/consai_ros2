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

"""試合状況を統合的に表現するWorldModelの定義モジュール."""

from dataclasses import dataclass

from consai_game.world_model.ball_activity_model import BallActivityModel
from consai_game.world_model.ball_model import BallModel
from consai_game.world_model.ball_position_model import BallPositionModel
from consai_game.world_model.field_model import Field, FieldPoints
from consai_game.world_model.referee_model import RefereeModel
from consai_game.world_model.robot_activity_model import RobotActivityModel
from consai_game.world_model.robots_model import RobotsModel
from consai_game.world_model.kick_target_model import KickTargetModel
from consai_game.world_model.game_config_model import GameConfigModel
from consai_game.world_model.threats_model import ThreatsModel
from consai_game.world_model.world_meta_model import WorldMetaModel


@dataclass
class WorldModel:
    """試合全体の状態を統合的に保持するデータクラス."""

    referee: RefereeModel = RefereeModel()
    robots: RobotsModel = RobotsModel()
    ball: BallModel = BallModel()
    field: Field = Field()
    field_points: FieldPoints = FieldPoints.create_field_points(field)
    ball_position: BallPositionModel = BallPositionModel(field, field_points)
    robot_activity: RobotActivityModel = RobotActivityModel()
    ball_activity: BallActivityModel = BallActivityModel()
    kick_target: KickTargetModel = KickTargetModel()
    game_config: GameConfigModel = GameConfigModel()
    threats: ThreatsModel = ThreatsModel(field, field_points)
    meta: WorldMetaModel = WorldMetaModel()
