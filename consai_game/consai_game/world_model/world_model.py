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
from dataclasses import field as dataclass_field

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

    referee: RefereeModel = dataclass_field(default_factory=RefereeModel)
    robots: RobotsModel = dataclass_field(default_factory=RobotsModel)
    ball: BallModel = dataclass_field(default_factory=BallModel)
    field: Field = dataclass_field(default_factory=Field)
    robot_activity: RobotActivityModel = dataclass_field(default_factory=RobotActivityModel)
    ball_activity: BallActivityModel = dataclass_field(default_factory=BallActivityModel)
    kick_target: KickTargetModel = dataclass_field(default_factory=KickTargetModel)
    game_config: GameConfigModel = dataclass_field(default_factory=GameConfigModel)
    meta: WorldMetaModel = dataclass_field(default_factory=WorldMetaModel)

    field_points: FieldPoints = dataclass_field(init=False)
    ball_position: BallPositionModel = dataclass_field(init=False)
    threats: ThreatsModel = dataclass_field(init=False)

    def __post_init__(self):
        """クラスメンバを使って初期化を行う"""
        self.field_points = FieldPoints.create_field_points(self.field)
        self.ball_position = BallPositionModel(self.field, self.field_points)
        self.threats = ThreatsModel(self.field, self.field_points)
