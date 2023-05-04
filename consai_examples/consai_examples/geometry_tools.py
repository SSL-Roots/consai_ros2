#!/usr/bin/env python3
# coding: UTF-8

# Copyright 2021 Roots
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

import math
import cmath
import numpy

from consai_msgs.msg import State2D

def get_diff_xy(pose1, pose2):

    diff_pose = State2D()

    # 2点間の距離(xとy)を取る関数
    diff_pose.x = pose1.x - pose2.x
    diff_pose.y = pose1.y - pose2.y

    return diff_pose

def get_distance(pose1, pose2):
    # 2点間の距離を取る関数
    diff = get_diff_xy(pose1, pose2)

    return math.hypot(diff.x, diff.y)

def get_line_parameter(pose1, pose2):
    # 2点間を結ぶ直線の傾きと切片を求める関数
    diff = get_diff_xy(pose1, pose2)
   
    # 直線の傾き
    if diff.x == 0:
        slope = diff.x
    else:
        slope = diff.y / diff.x

    # 直線の切片
    intercept = pose2.y - slope * pose2.x

    return slope, intercept

def get_angle(from_pose, to_pose):
    # ワールド座標系でfrom_poseからto_poseを結ぶ直線の角度を得る
    diff_pose = get_diff_xy(to_pose, from_pose)

    return math.atan2(diff_pose.y, diff_pose.x)

def angle_normalize(angle):
    # 角度をpi  ~ -piの範囲に変換する関数
    while angle > math.pi:
        angle -= 2*math.pi

    while angle < -math.pi:
        angle += 2*math.pi

    return angle

class Trans():
    # 座標系を移動、回転するクラス
    def __init__(self, center , theta):
        normalized_theta = angle_normalize(theta)
        self._c_center = center.x + center.y * 1.0j
        self._c_rotate = cmath.rect(1.0, normalized_theta) 
        self._c_angle = normalized_theta

    def transform(self, pose):
        c_point = pose.x + pose.y * 1.0j
        c_output = (c_point - self._c_center) * numpy.conj(self._c_rotate)

        _pose = State2D()
        _pose.x = c_output.real
        _pose.y = c_output.imag
        return _pose

    def inverted_transform(self, pose):
        c_point = pose.x + pose.y * 1.0j
        c_output = c_point * self._c_rotate + self._c_center

        _pose = State2D()
        _pose.x = c_output.real
        _pose.y = c_output.imag
        return _pose

    def transform_angle(self, angle):
        return angle_normalize(angle - self._c_angle)

    def inverted_transform_angle(self, angle):
        return angle_normalize(angle + self._c_angle)

