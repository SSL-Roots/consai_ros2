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

"""予測を統合したPerceptionの定義モジュール."""

from dataclasses import dataclass

from consai_game.world_model.perception.ball_decision import BallDecision
from consai_game.world_model.perception.ball_prediction import BallPrediction
from consai_game.world_model.perception.robot_decision import RobotDecision


@dataclass
class Perception:
    """予測に関する関数やクラスを統合的に保持するデータクラス."""

    ball_decision: BallDecision = BallDecision()
    ball_prediction: BallPrediction = BallPrediction()
    robot_decision: RobotDecision = RobotDecision()
