#!/usr/bin/env python3
# coding: UTF-8
#
# Copyright 2023 Roots
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

from consai_msgs.msg import State2D
from consai_tools.geometry import geometry_tools as tool
import math
import pytest


def test_trans_class():

    pose1 = State2D(x=0.0, y=0.0)
    pose2 = State2D(x=0.0, y=1.0)
    trans = tool.Trans(pose1, tool.get_angle(pose1, pose2))
    p_trans = trans.transform(pose2)
    assert math.isclose(p_trans.x, 1, abs_tol=0.01)
    assert math.isclose(p_trans.y, 0, abs_tol=0.01)

    p = trans.inverted_transform(p_trans)
    assert math.isclose(p.x, 0, abs_tol=0.01)
    assert math.isclose(p.y, 1, abs_tol=0.01)

    pose1 = State2D(x=0.0, y=0.0)
    pose2 = State2D(x=0.0, y=1.0)
    trans = tool.Trans(pose1, tool.get_angle(pose1, pose2))
    p_trans = trans.transform(pose2)
    assert math.isclose(p_trans.x, 1, abs_tol=0.01)
    assert math.isclose(p_trans.y, 0, abs_tol=0.01)

    p = trans.inverted_transform(p_trans)
    assert math.isclose(p.x, 0, abs_tol=0.01)
    assert math.isclose(p.y, 1, abs_tol=0.01)


def test_get_diff_xy():

    pose1 = State2D(x=0.0, y=0.0)
    pose2 = State2D(x=1.0, y=1.0)

    diff_pose = tool.get_diff_xy(pose1, pose2)
    assert math.isclose(diff_pose.x, -1.0, abs_tol=0.01)
    assert math.isclose(diff_pose.y, -1.0, abs_tol=0.01)

    pose1 = State2D(x=-2.0, y=0.0)
    pose2 = State2D(x=0.0, y=3.0)

    diff_pose = tool.get_diff_xy(pose1, pose2)
    assert math.isclose(diff_pose.x, -2.0, abs_tol=0.01)
    assert math.isclose(diff_pose.y, -3.0, abs_tol=0.01)

    pose1 = State2D(x=2.0, y=5.0)
    pose2 = State2D(x=0.0, y=2.0)

    diff_pose = tool.get_diff_xy(pose1, pose2)
    assert math.isclose(diff_pose.x, 2.0, abs_tol=0.01)
    assert math.isclose(diff_pose.y, 3.0, abs_tol=0.01)


def test_get_distance():

    pose1 = State2D(x=0.0, y=0.0)
    pose2 = State2D(x=1.0, y=1.0)
    distance = tool.get_distance(pose1, pose2)
    assert math.isclose(distance, 1.414, abs_tol=0.01)

    pose1 = State2D(x=-1.0, y=2.0)
    pose2 = State2D(x=1.0, y=1.0)
    distance = tool.get_distance(pose1, pose2)
    assert math.isclose(distance, 2.236, abs_tol=0.01)


params = {
    '0': (State2D(x=0.0, y=0.0), State2D(x=1.0, y=1.0), 1, 0, True),
    '1': (State2D(x=-1.0, y=0.0), State2D(x=1.0, y=1.0), 1/2, 1/2, True),
    '2': (State2D(x=1.0, y=0.0), State2D(x=-1.0, y=2.0), -1, 1, True),
    'y=b': (State2D(x=-1.0, y=4.0), State2D(x=1.0, y=4.0), 0, 4, True),
    'x=c': (State2D(x=1.0, y=0.0), State2D(x=1.0, y=2.0), None, None, False)
}


@pytest.mark.parametrize("pose1, pose2, expect_slope, expect_intercept, expect_flag",
                         list(params.values()), ids=list(params.keys()))
def test_get_line_parameter(pose1, pose2, expect_slope, expect_intercept, expect_flag):

    actual_slope, actual_intercept, actual_flag = tool.get_line_parameter(pose1, pose2)

    if actual_flag:
        assert math.isclose(actual_slope, expect_slope, abs_tol=0.01)
        assert math.isclose(actual_intercept, expect_intercept, abs_tol=0.01)
        assert actual_flag == expect_flag
    else:
        assert actual_slope is None
        assert actual_intercept is None
        assert actual_flag == expect_flag


def test_get_angle():
    from_pose = State2D(x=0.0, y=0.0)
    to_pose = State2D(x=1.0, y=1.0)
    angle = tool.get_angle(from_pose, to_pose)
    assert math.isclose(angle, float(math.pi / 4), abs_tol=0.01)

    from_pose = State2D(x=0.0, y=0.0)
    to_pose = State2D(x=-1.0, y=-1.0)
    angle = tool.get_angle(from_pose, to_pose)
    assert math.isclose(angle, float(-math.pi * 3 / 4), abs_tol=0.01)


def test_angle_normalize():
    angle = tool.angle_normalize(math.pi / 2)
    assert math.isclose(angle, float(math.pi / 2), abs_tol=0.01)

    angle = tool.angle_normalize(-math.pi / 2)
    assert math.isclose(angle, float(-math.pi / 2), abs_tol=0.01)

    angle = tool.angle_normalize(3 * math.pi)
    assert math.isclose(angle, float(math.pi), abs_tol=0.01)

    angle = tool.angle_normalize(-3 * math.pi)
    assert math.isclose(angle, float(-math.pi), abs_tol=0.01)
