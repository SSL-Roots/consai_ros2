#!/usr/bin/env python3
# coding: UTF-8

# Copyright 2024 Roots
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

"""DetectionWrapperのユニットテストモジュール."""

from consai_examples.observer.detection_wrapper import DetectionWrapper

import pytest

from tracked_frame_wrapper import TrackedFrameWrapper


def test_initialize():
    """DetectionWrapperの初期状態をテストする関数."""
    detection = DetectionWrapper(our_team_is_yellow=False)

    assert len(detection.our_robots()) == 0
    assert len(detection.their_robots()) == 0
    assert detection.ball().pos().x == pytest.approx(0.0)
    assert detection.ball().vel().x == pytest.approx(0.0)


def test_set_our_robots():
    """味方ロボットの設定処理をテストする関数."""
    detection = DetectionWrapper(our_team_is_yellow=False)

    frame = TrackedFrameWrapper()
    frame.append_valid_robots(blue_robots=[1, 2, 7], yellow_robots=[])
    detection.update(frame.get_frame())

    assert len(detection.our_robots()) == 3
    assert len(detection.their_robots()) == 0

    frame.append_valid_robots(blue_robots=[], yellow_robots=[0, 3])
    detection.update(frame.get_frame())

    assert len(detection.our_robots()) == 3
    assert len(detection.their_robots()) == 2


def test_change_team_color():
    """チームカラーの変更によるロボット識別の変化をテストする関数."""
    detection = DetectionWrapper(our_team_is_yellow=True)

    frame = TrackedFrameWrapper()
    frame.append_valid_robots(blue_robots=[1], yellow_robots=[2, 3, 4, 5])
    detection.update(frame.get_frame())

    assert len(detection.our_robots()) == 4
    assert len(detection.their_robots()) == 1


def test_set_ball():
    """ボールの位置設定処理をテストする関数."""
    detection = DetectionWrapper(our_team_is_yellow=False)

    frame = TrackedFrameWrapper()
    frame.append_valid_ball(1.2, 3.4)
    detection.update(frame.get_frame())

    assert detection.ball().pos().x == pytest.approx(1.2)
    assert detection.ball().pos().y == pytest.approx(3.4)


def test_invalid_robots():
    """無効なロボットが設定された場合の挙動をテストする関数."""
    detection = DetectionWrapper(our_team_is_yellow=False)

    frame = TrackedFrameWrapper()
    frame.append_invalid_robots(blue_robots=[1, 2, 7], yellow_robots=[0, 1, 2])
    detection.update(frame.get_frame())

    assert len(detection.our_robots()) == 0
    assert len(detection.their_robots()) == 0
