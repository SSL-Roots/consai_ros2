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

def get_distance(pose1, pose2):
    # 2点間の距離を取る関数

    diff_pose_x = pose1[0] - pose2[0]
    diff_pose_y = pose1[1] - pose2[1]

    return math.hypot(diff_pose_x, diff_pose_y)

def get_angle(from_pose, to_pose):
    # ワールド座標系でfrom_poseからto_poseを結ぶ直線の角度を得る

    diff_pose_x = to_pose[0] - from_pose[0]
    diff_pose_y = to_pose[1] - from_pose[1]

    return math.atan2(diff_pose_y, diff_pose_x)

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
        self._c_center = center[0] + center[1] * 1.0j
        self._c_rotate = cmath.rect(1.0, normalized_theta) 
        self._c_angle = normalized_theta

    def transform(self, pose):
        c_point = pose[0] + pose[1] * 1.0j
        c_output = (c_point - self._c_center) * numpy.conj(self._c_rotate)

        return [c_output.real, c_output.imag]

    def inverted_transform(self, pose):
        c_point = pose[0] + pose[1] * 1.0j
        c_output = c_point * self._c_rotate + self._c_center

        return [c_output.real, c_output.imag]

    def transform_angle(self, angle):
        return angle_normalize(angle - self._c_angle)

    def inverted_transform_angle(self, angle):
        return angle_normalize(angle + self._c_angle)

