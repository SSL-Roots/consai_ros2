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


def get_diff_xy(pose1: State2D, pose2: State2D) -> State2D:

    diff_pose = State2D()

    # 2点間の距離(xとy)を取る関数
    diff_pose.x = pose1.x - pose2.x
    diff_pose.y = pose1.y - pose2.y

    return diff_pose


def get_distance(pose1: State2D, pose2: State2D) -> float:
    # 2点間の距離を取る関数
    diff = get_diff_xy(pose1, pose2)

    return math.hypot(diff.x, diff.y)


def get_norm(pose: State2D) -> float:
    # 大きさを取る関数
    return math.hypot(pose.x, pose.y)


def get_line_parameter(pose1, pose2):
    # 2点間を結ぶ直線の傾きと切片を求める関数
    diff = get_diff_xy(pose1, pose2)

    # 計算できた場合Ture
    flag = True

    # 2点が縦に並ぶ場合
    if math.isclose(diff.x, 0.0):
        slope = None
        intercept = None
        flag = False
    # 2点が横に並ぶ場合
    elif math.isclose(diff.y, 0.0):
        slope = 0
        intercept = pose1.y
    else:
        # 直線の傾き
        slope = diff.y / diff.x
        # 直線の切片
        intercept = pose2.y - slope * pose2.x

    return slope, intercept, flag


def get_angle(from_pose, to_pose):
    # ワールド座標系でfrom_poseからto_poseを結ぶ直線の角度を得る
    diff_pose = get_diff_xy(to_pose, from_pose)

    return math.atan2(diff_pose.y, diff_pose.x)


def get_vel_angle(vel: State2D) -> float:
    # ワールド座標系での速度ベクトルの角度を得る
    return math.atan2(vel.y, vel.x)


def angle_normalize(angle):
    # 角度をpi  ~ -piの範囲に変換する関数
    while angle > math.pi:
        angle -= 2 * math.pi

    while angle < -math.pi:
        angle += 2 * math.pi

    return angle


def is_on_line(pose: State2D, line_pose1: State2D, line_pose2: State2D, tolerance: float) -> bool:
    # poseがline_pose1とline_pose2を結ぶ直線上にあるかを判定する関数
    # toleranceは許容誤差

    trans_p1_to_p2 = Trans(line_pose1, get_angle(line_pose1, line_pose2))
    pose_P1toP2 = trans_p1_to_p2.transform(pose)
    p2_P1toP2 = trans_p1_to_p2.transform(line_pose2)

    if pose_P1toP2.x < 0.0:
        return False

    if pose_P1toP2.x > p2_P1toP2.x:
        return False

    if abs(pose_P1toP2.y) > tolerance:
        return False

    return True


class Trans:
    # 座標系を移動、回転するクラス
    def __init__(self, center, theta):
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


def get_line_intersection(
    line1_pose1: State2D, line1_pose2: State2D, line2_pose1: State2D, line2_pose2: State2D
) -> State2D:
    # 2つの線分の交点を計算する関数

    # 1つ目の線分の傾きと切片を計算
    slope1, intercept1, flag1 = get_line_parameter(line1_pose1, line1_pose2)
    # 2つ目の線分の傾きと切片を計算
    slope2, intercept2, flag2 = get_line_parameter(line2_pose1, line2_pose2)

    # どちらかの線分が垂直な場合
    if not flag1 or not flag2:
        # 両方の線分が垂直な場合
        if not flag1 and not flag2:
            return None
        # 1つ目の線分が垂直な場合
        if not flag1:
            x = line1_pose1.x
            y = slope2 * x + intercept2
        # 2つ目の線分が垂直な場合
        else:
            x = line2_pose1.x
            y = slope1 * x + intercept1
    else:
        # 両方の線分が垂直でない場合
        # 傾きが同じ場合（平行）
        if math.isclose(slope1, slope2):
            return None
        # 交点のx座標を計算
        x = (intercept2 - intercept1) / (slope1 - slope2)
        # 交点のy座標を計算
        y = slope1 * x + intercept1

    # 交点が両方の線分上にあるかチェック
    if not is_on_line(State2D(x=x, y=y), line1_pose1, line1_pose2, 0.001):
        return None
    if not is_on_line(State2D(x=x, y=y), line2_pose1, line2_pose2, 0.001):
        return None

    return State2D(x=x, y=y)
