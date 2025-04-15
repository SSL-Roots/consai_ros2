#!/usr/bin/env python3
# coding: UTF-8

"""
フィールドの寸法およびフィールド上の各ポイントを定義するクラスを提供する.

各クラスはフィールドの基本情報（長さ・幅・ゴールの大きさ）やフィールド上の重要な座標を保持・生成する.
"""

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


from dataclasses import dataclass

from consai_game.utils.geometry import Point, Rectangle


@dataclass
class Field:
    """フィールド寸法を保持するクラス."""

    length: float = 12.0
    width: float = 9.0
    goal_width: float = 1.8
    penalty_depth: float = 1.8
    penalty_width: float = 3.6

    half_length = length / 2
    half_width = width / 2
    half_goal_width = goal_width / 2
    half_penalty_depth = penalty_depth / 2
    half_penalty_width = penalty_width / 2


@dataclass
class FieldPoints:
    """フィールドの各ポイントを保持するクラス."""

    center: Point
    our_goal_top: Point
    our_goal_bottom: Point
    their_goal_top: Point
    their_goal_bottom: Point
    corners: Rectangle
    our_defense_area: Rectangle
    their_defense_area: Rectangle

    @staticmethod
    def create_field_points(field: Field) -> "FieldPoints":
        """フィールドの各ポイントを生成するメソッド."""
        center = Point(0.0, 0.0)

        half_length = field.length / 2
        half_width = field.width / 2
        half_goal_width = field.goal_width / 2

        our_goal_top = Point(-half_length, half_goal_width)
        our_goal_bottom = Point(-half_length, -half_goal_width)
        their_goal_top = Point(half_length, half_goal_width)
        their_goal_bottom = Point(half_length, -half_goal_width)

        corners = Rectangle(
            top_left=Point(-half_length, half_width),
            top_right=Point(half_length, half_width),
            bottom_left=Point(-half_length, -half_width),
            bottom_right=Point(half_length, -half_width),
        )

        half_penalty_width = field.penalty_width / 2
        defense_area_length_inside = half_length - field.penalty_depth
        defense_area_length_outside = half_length
        our_defense_area = Rectangle(
            top_left=Point(-defense_area_length_outside, half_penalty_width),
            top_right=Point(-defense_area_length_inside, half_penalty_width),
            bottom_left=Point(-defense_area_length_outside, -half_penalty_width),
            bottom_right=Point(-defense_area_length_inside, -half_penalty_width),
        )
        their_defense_area = Rectangle(
            top_left=Point(defense_area_length_inside, half_penalty_width),
            top_right=Point(defense_area_length_outside, half_penalty_width),
            bottom_left=Point(defense_area_length_inside, -half_penalty_width),
            bottom_right=Point(defense_area_length_outside, -half_penalty_width),
        )
        return FieldPoints(
            center=center,
            our_goal_top=our_goal_top,
            our_goal_bottom=our_goal_bottom,
            their_goal_top=their_goal_top,
            their_goal_bottom=their_goal_bottom,
            corners=corners,
            our_defense_area=our_defense_area,
            their_defense_area=their_defense_area,
        )
